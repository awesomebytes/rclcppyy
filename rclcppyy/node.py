import inspect
import re
import uuid
from typing import Union, Optional, Callable, Any, List
import rclpy
from rclpy.callback_groups import CallbackGroup
from rclpy.event_handler import PublisherEventCallbacks, SubscriptionEventCallbacks
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy, LivelinessPolicy
from rclpy.context import Context
from rclpy.qos_overriding_options import QoSOverridingOptions
from rclpy.exceptions import InvalidTopicNameException
from rclpy.validate_topic_name import validate_topic_name
from rclpy.parameter import Parameter
import cppyy
from rclcppyy import bringup_rclcpp
from rclcppyy.bringup_rclcpp import _resolve_message_type, _is_msg_python, convert_python_msg_to_cpp


def _subscription_header(cpp_type_str):
    """C++ header path for a resolved message type string (e.g.
    ``sensor_msgs::msg::Image`` -> ``sensor_msgs/msg/image.hpp``), for the
    subscription cache's ``#include``. Derived from the type string because under
    acceleration the message class is already a C++ (cppyy) type, for which
    rclcpp_kit.message_header returns None. Uses rosidl's CamelCase->snake_case
    convention (``PointCloud2`` -> ``point_cloud2``, ``TFMessage`` -> ``tf_message``).
    Returns None if the string isn't a ``pkg::msg::Name`` triple.
    """
    parts = cpp_type_str.split("::")
    if len(parts) != 3:
        return None
    package, _, name = parts
    snake = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2",
                   re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)).lower()
    return "%s/msg/%s.hpp" % (package, snake)


class RclcppyyNode(Node):
    """
    This node inherits from rclpy Node but it uses rclcpp for publishers/subscribers.
    """
    def __init__(self,
        node_name: str,
        *,
        context: Optional[Context] = None,
        cli_args: Optional[List[str]] = None,
        namespace: Optional[str] = None,
        use_global_arguments: bool = True,
        enable_rosout: bool = True,
        start_parameter_services: bool = True,
        parameter_overrides: Optional[List[Parameter]] = None,
        allow_undeclared_parameters: bool = False,
        automatically_declare_parameters_from_overrides: bool = False,
        enable_logger_service: bool = False) -> None:
        # Before creating the rclpy node, we create the rclcpp node (as
        # internally it calls create_publisher for the parameters)
        self._rclcpp = bringup_rclcpp()
        # We need to init rclcpp if we havent already
        if not self._rclcpp.ok():
            self._rclcpp.init()

        # Just cause its annoying, initialise if we havent yet
        if not rclpy.ok():
            rclpy.init()
        
        self._rclcpp_node = self._rclcpp.Node(node_name + "_rclcpp")
        self._cpp_publishers = {}
        
        # Define the C++ callback wrapper template for timers
        # cppyy.cppdef("""
        #     #include <Python.h>
        #     #include <functional>
            
        #     static std::function<void()> create_node_timer_callback_PLACEHOLDER_UUID(PyObject* self) {
        #         return [self]() {
        #             if (self && PyObject_HasAttrString(self, "_node_timer_callback_PLACEHOLDER_UUID")) {
        #                 PyObject_CallMethod(self, "_node_timer_callback_PLACEHOLDER_UUID", nullptr);
        #             }
        #         };
        #     }
        # """)

        self._timer_cpp_template = """
            #include <Python.h>
            #include <functional>
            
            static std::function<void()> create_node_timer_callback_PLACEHOLDER_UUID(PyObject* self) {
                return [self]() {
                    if (self && PyObject_HasAttrString(self, "_node_timer_callback_PLACEHOLDER_UUID")) {
                        PyObject_CallMethod(self, "_node_timer_callback_PLACEHOLDER_UUID", nullptr);
                    }
                };
            }
        """
        self._cpp_timers = {}

        self._cpp_subscriptions = {}
        
        # Now we create the rclpy node
        super().__init__(node_name,
                         context=context,
                         cli_args=cli_args,
                         namespace=namespace,
                         use_global_arguments=use_global_arguments,
                         enable_rosout=enable_rosout,
                         start_parameter_services=start_parameter_services,
                         parameter_overrides=parameter_overrides,
                         allow_undeclared_parameters=allow_undeclared_parameters,
                         automatically_declare_parameters_from_overrides=(
                             automatically_declare_parameters_from_overrides
                         ),
                         enable_logger_service=enable_logger_service)

    # TODO: deal with spinners, if spinning on one, we should spin on both

    def _convert_qos_to_rclcpp(self, qos: QoSProfile) -> 'rclcpp.QoS':
        """
        Convert a rclpy QoSProfile to a rclcpp QoS.
        """
        rclcpp_qos = self._rclcpp.QoS(qos.depth)
        
        # History policy
        if qos.history == HistoryPolicy.KEEP_LAST:
            rclcpp_qos.keep_last(qos.depth)
        elif qos.history == HistoryPolicy.KEEP_ALL:
            rclcpp_qos.keep_all()
            
        # Reliability
        if qos.reliability == ReliabilityPolicy.RELIABLE:
            rclcpp_qos.reliable()
        elif qos.reliability == ReliabilityPolicy.BEST_EFFORT:
            rclcpp_qos.best_effort()
            
        # Durability
        if qos.durability == DurabilityPolicy.TRANSIENT_LOCAL:
            rclcpp_qos.transient_local()
        elif qos.durability == DurabilityPolicy.VOLATILE:
            rclcpp_qos.durability_volatile()
            
        # Liveliness
        if qos.liveliness == LivelinessPolicy.AUTOMATIC:
            rclcpp_qos.liveliness(self._rclcpp.LivelinessPolicy.Automatic)
        elif qos.liveliness == LivelinessPolicy.MANUAL_BY_TOPIC:
            rclcpp_qos.liveliness(self._rclcpp.LivelinessPolicy.ManualByTopic)
            
        # Deadline
        if qos.deadline is not None:
            # Translate rclpy deadline (Duration) to rclcpp::Duration
            rclcpp_qos.deadline(self._rclcpp.Duration(qos.deadline.nanoseconds))
            
        # Lifespan
        if qos.lifespan is not None:
            # Translate rclpy lifespan (Duration) to rclcpp::Duration
            rclcpp_qos.lifespan(self._rclcpp.Duration(qos.lifespan.nanoseconds))
            
        # Lease Duration
        if qos.liveliness_lease_duration is not None:
            # Translate rclpy liveliness lease duration (Duration) to rclcpp::Duration
            rclcpp_qos.liveliness_lease_duration(self._rclcpp.Duration(qos.liveliness_lease_duration.nanoseconds))
            
        return rclcpp_qos

    def create_publisher(self, 
                         msg_type: Any,
                         topic: str, 
                         qos_profile: QoSProfile | int,
                           *,
                        callback_group: CallbackGroup | None = None,
                        event_callbacks: PublisherEventCallbacks | None = None,
                        qos_overriding_options: QoSOverridingOptions | None = None, 
                        publisher_class: type[Publisher] = ...) -> Publisher:
        # Validate and process inputs like rclpy does
        qos_profile = self._validate_qos_or_depth_parameter(qos_profile)

        # Resolve the topic name
        try:
            final_topic = self.resolve_topic_name(topic)
        except RuntimeError:
            try:
                self._validate_topic_or_service_name(topic)
            except InvalidTopicNameException as ex:
                raise ex from None
            raise

        # Handle QoS parameter overrides if needed
        # if qos_overriding_options is None:
        #     qos_overriding_options = QoSOverridingOptions([])
        # _declare_qos_parameters(
        #     Publisher, self, final_topic, qos_profile, qos_overriding_options)

        # Convert rclpy message to rclcpp message type
        rclcpp_msg_type, rclcpp_msg_class = _resolve_message_type(msg_type)

        # Create publisher options for rclcpp
        # Note: not fully sure if this is correct, but it works for now
        options = self._rclcpp.PublisherOptionsWithAllocator["std::allocator<void>"]()
        if callback_group:
            # TODO: if we get a callback group from rclpy, we need to convert it to rclcpp
            # Here we do nothing, so the default callback group from rclcpp Node is used
            pass
        if qos_overriding_options:
            # options.qos_overriding_options = qos_overriding_options
            # TODO: implement this
            pass
        
        # Convert QoS profile to rclcpp QoS
        rclcpp_qos = self._convert_qos_to_rclcpp(qos_profile)
        
        # Create the actual rclcpp publisher
        publisher = self._rclcpp.create_publisher[rclcpp_msg_type](
            self._rclcpp_node,
            final_topic,
            rclcpp_qos,
            options
        )

        # Wrap in rclpy publisher for compatibility if needed
        if event_callbacks:
            # TODO: Handle event callbacks through rclpy wrapper
            pass

        self._publishers.append(publisher)
        self._cpp_publishers[publisher] = publisher
        # Wake executor like rclpy does
        self._wake_executor()

        # Only wrap publish method if the message type is Python (which shouldn't happen with proper import hooks)
        if _is_msg_python(msg_type):
            # Replace the publish method so we translate the message to rclcpp message if needed
            original_publish = publisher.publish
            def publish_wrapper(msg: Any) -> Any:
                """
                Make sure that when calling publish, the message is converted to rclcpp message if needed
                """
                if _is_msg_python(msg):
                    print(f"Converting message to C++: {type(msg)}")
                    _, rclcpp_msg_class = _resolve_message_type(msg)
                    # Inefficient, but it works, ideally we should monkeypatch
                    # the imported messages to convert them to cpp from the start
                    cpp_msg = rclcpp_msg_class()
                    cpp_msg = convert_python_msg_to_cpp(msg, cpp_msg)
                else:
                    cpp_msg = msg
                return original_publish(cpp_msg)
                
            publisher.publish = publish_wrapper
        # else: no wrapping needed, publish directly uses C++ messages
        
        return publisher

    def create_timer(
        self, period: float, callback: Callable[[], None], *,
        oneshot: bool = False, callback_group: CallbackGroup | None = None
    ):
        """
        Create a timer using the C++ rclcpp timer.
        """
        # Convert period to nanoseconds for rclcpp  
        period_ns = int(period * 1e9)
        # Generate a unique UUID for the timer
        uuid_str = str(uuid.uuid4()).replace("-", "_")
        # Replace the PLACEHOLDER_UUID in the C++ template with the actual UUID
        cpp_timer_template = self._timer_cpp_template.replace("PLACEHOLDER_UUID", uuid_str)
        # Compile the C++ callback wrapper
        cppyy.cppdef(cpp_timer_template)
        # Add dynamically a method to the node that will call the callback
        setattr(self, f"_node_timer_callback_{uuid_str}", callback)

        # Create the C++ callback wrapper
        cpp_callback = getattr(cppyy.gbl, f"create_node_timer_callback_{uuid_str}")(self)
        
        # Create the timer using the rclcpp node's create_wall_timer
        wall_timer = self._rclcpp_node.create_wall_timer(
            cppyy.gbl.std.chrono.nanoseconds(period_ns),
            cpp_callback
        )
        
        # TODO: Handle oneshot and callback_group parameters if needed
        # For now, we ignore these parameters as the provided implementation doesn't handle them
        
        return wall_timer

    def create_subscription(self, 
                            msg_type: Any,
                            topic: str, 
                            callback: Callable[[Any], None],
                            qos_profile: QoSProfile | int,
                            *,
                            callback_group: CallbackGroup | None = None,
                            event_callbacks: SubscriptionEventCallbacks | None = None,
                            qos_overriding_options: QoSOverridingOptions | None = None,
                            raw: bool = False) -> Any:
        """
        Create a subscription using the C++ rclcpp subscription.
        """
        # Validate and process inputs like rclpy does
        qos_profile = self._validate_qos_or_depth_parameter(qos_profile)

        # Resolve the topic name
        try:
            final_topic = self.resolve_topic_name(topic)
        except RuntimeError:
            try:
                self._validate_topic_or_service_name(topic)
            except InvalidTopicNameException as ex:
                raise ex from None
            raise

        # Convert rclpy message to rclcpp message type
        rclcpp_msg_type, rclcpp_msg_class = _resolve_message_type(msg_type)

        # Build the C++ callback directly as a std::function. cppyy delivers the C++
        # message proxy straight to the Python callback and manages the GIL, so no
        # per-subscription C++ trampoline (the old per-UUID <Python.h> lambda that
        # rebound a raw pointer) is needed.
        cpp_callback = cppyy.gbl.std.function[
            "void(std::shared_ptr<const %s>)" % rclcpp_msg_type
        ](callback)

        # Convert QoS profile to rclcpp QoS
        rclcpp_qos = self._convert_qos_to_rclcpp(qos_profile)

        # Route through the cached per-type trampoline so the ~2.8 s
        # create_subscription<MsgT> template instantiation JITs once into a .so
        # instead of on every process start. Falls back to the plain template call
        # when no .so is built yet (this run), for callback groups / QoS overrides,
        # or when the message header can't be derived -- correctness first, the cache
        # is a pure speedup.
        subscription = None
        if callback_group is None and qos_overriding_options is None:
            try:
                # subscription_cache postdates rclcpp_kit 0.1.0; on an older suite it
                # is absent and we simply use the plain path.
                from rclcpp_kit import subscription_cache
                header = _subscription_header(rclcpp_msg_type)
                if header:
                    subscription = subscription_cache.make_subscription(
                        self._rclcpp_node, rclcpp_msg_type, header, final_topic,
                        rclcpp_qos, cpp_callback)
            except ImportError:
                subscription = None
        if subscription is None:
            options = self._rclcpp.SubscriptionOptionsWithAllocator["std::allocator<void>"]()
            subscription = self._rclcpp.create_subscription[rclcpp_msg_type](
                self._rclcpp_node, final_topic, rclcpp_qos, cpp_callback, options)

        # Keep the Python callback and its std::function wrapper alive for the
        # subscription's lifetime; dropping them would dangle the C++ callback.
        self._cpp_subscriptions[subscription] = {
            'callback': callback,
            'cpp_callback': cpp_callback,
            'msg_type': rclcpp_msg_type,
        }

        # Add to subscriptions list for compatibility
        self._subscriptions.append(subscription)

        # Wake executor like rclpy does
        self._wake_executor()

        return subscription

    def destroy_node(self):
        """
        Destroy the node.

        The publishers/subscriptions this node creates are rclcpp (cppyy) objects
        that we keep in the rclpy ``self._publishers`` / ``self._subscriptions``
        lists for API compatibility. rclpy's ``destroy_publisher`` /
        ``destroy_subscription`` assume rclpy objects (e.g. ``.event_handlers``),
        so let RAII reclaim the C++ entities and drop them from the rclpy
        bookkeeping before delegating to the rclpy node teardown.
        """
        self._publishers.clear()
        self._subscriptions.clear()
        self._cpp_publishers.clear()
        self._cpp_subscriptions.clear()
        self._cpp_timers.clear()
        super().destroy_node()

    def _validate_topic_or_service_name(self, name: str) -> None:
        """
        Validate a topic or service name.
        """
        validate_topic_name(name)

    def _validate_qos_or_depth_parameter(self, qos_profile: Union[QoSProfile, int]) -> QoSProfile:
        """
        Validate a QoS profile or convert a history depth to a QoSProfile.
        """
        if isinstance(qos_profile, QoSProfile):
            return qos_profile
        if isinstance(qos_profile, int):
            return QoSProfile(depth=qos_profile)
        raise TypeError('qos_profile should be a QoSProfile or an integer')

    def _wake_executor(self) -> None:
        """
        Wake the executor if one exists.
        """
        # TODO: Implement waking the executor when we have one
        pass

def enable_kwargs_for_cpp_msg(msg_class):
    """Enable keyword arguments for C++ message constructors"""
    original_init = msg_class.__init__
    def new_init(self, *args, **kwargs):
        if args:
            original_init(self, *args)
        else:
            original_init(self)
        
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
            else:
                raise AttributeError(f"'{self.__class__.__name__}' object has no attribute '{key}'")
    
    msg_class.__init__ = new_init

def add_python_msg_sugar_to_cpp_msg(cpp_msg_class, py_msg_class):
    """
    Python messages pretty print and also you can check the fields and field
    types, so lets forward that to the C++ messages
    """
    cpp_msg_class.__repr__ = py_msg_class.__repr__
    cpp_msg_class.SLOT_TYPES = py_msg_class.SLOT_TYPES
    cpp_msg_class.get_fields_and_field_types = py_msg_class.get_fields_and_field_types
    cpp_msg_class._TYPE_SUPPORT = py_msg_class._TYPE_SUPPORT

    # add a fallback to the Python message if an attribute is not found
    def fallback_getattr(self, name):
        print(f"Called fallback_getattr with name: {name}")
        if not hasattr(self, name):
            return getattr(py_msg_class, name)
        return super().__getattr__(name)
    cpp_msg_class.__getattr__ = fallback_getattr


def convert_python_msgs_to_cpp(target_globals=None):
    """
    Convert all Python ROS messages in globals() to their C++ equivalents with keyword support.
    
    Args:
        target_globals: The globals dict to modify. If None, uses caller's globals().
    """
    if target_globals is None:
        # Get the caller's globals
        frame = inspect.currentframe().f_back
        target_globals = frame.f_globals
    
    # Find all Python ROS message classes in globals
    python_msgs = {}
    for name, obj in list(target_globals.items()):
        try:
            # Check if this is a Python ROS message class (not instance)
            if (hasattr(obj, '__module__') and 
                hasattr(obj, '__name__') and
                _is_msg_python(obj) and
                inspect.isclass(obj)):
                python_msgs[name] = obj
        except Exception:
            # Skip any objects that cause issues during inspection
            continue
    
    print(f"Found {len(python_msgs)} Python ROS message classes: {list(python_msgs.keys())}")
    
    # Convert each Python message to C++ equivalent
    for name, py_msg_class in python_msgs.items():
        try:
            # Get the C++ equivalent
            cpp_msg_type, cpp_msg_class = _resolve_message_type(py_msg_class)
            
            # Apply keyword argument monkey-patch
            enable_kwargs_for_cpp_msg(cpp_msg_class)
            
            # Replace in globals
            target_globals[name] = cpp_msg_class
            print(f"Converted {name}: {py_msg_class} -> {cpp_msg_class}")
            
        except Exception as e:
            print(f"Failed to convert {name}: {e}")
            # Keep the original Python version if conversion fails
            continue

def setup_automatic_cpp_conversion():
    """
    Set up automatic conversion of Python ROS messages to C++ equivalents.
    This sets up an import hook that will automatically convert messages during import.
    """
    import sys
    import importlib.util
    from importlib.abc import Loader, MetaPathFinder
    from importlib.machinery import ModuleSpec
    
    class ROSMessageConverter(MetaPathFinder):
        """Import hook to automatically convert ROS messages to C++ equivalents"""
        
        def __init__(self):
            # Keep track of modules we're currently importing to avoid recursion
            self._importing = set()
            # Store the original loaders we wrap
            self._original_loaders = {}
        
        def find_spec(self, fullname, path, target=None):
            """Called by Python's import system for each import statement"""
            
            # Only intercept ROS message type imports (e.g. std_msgs.msg._string)
            # The actual message types are in the _<message> modules
            if '.msg._' in fullname and fullname not in self._importing:
                # Let the normal import happen first
                self._importing.add(fullname)
                try:
                    spec = importlib.util.find_spec(fullname)
                finally:
                    self._importing.remove(fullname)
                
                if spec is not None and spec.loader is not None:
                    # Remember the original loader
                    self._original_loaders[fullname] = spec.loader
                    # Return a new spec with our custom loader
                    return ModuleSpec(
                        fullname,
                        ROSMessageLoader(self._original_loaders[fullname]),
                        origin=spec.origin,
                        is_package=spec.parent == ''
                    )
            return None
    
    class ROSMessageLoader(Loader):
        """Custom loader that converts ROS messages to C++ after normal loading"""
        
        def __init__(self, original_loader):
            self._original_loader = original_loader
        
        def create_module(self, spec):
            """Let the original loader create the module"""
            return self._original_loader.create_module(spec)
        
        def exec_module(self, module):
            """Execute the module normally, then convert any messages to C++"""
            # Execute with original loader first
            self._original_loader.exec_module(module)
            # Get the message name from the module dir(), we know it has an attr that is called 'Metaclass_MESSAGENAME'
            msg_name = [attr for attr in dir(module) if attr.startswith("Metaclass_")][0]
            msg_name = msg_name.replace("Metaclass_", "")
            # Only convert the actual message class
            if hasattr(module, msg_name):
                msg_class = getattr(module, msg_name)
                try:
                    if _is_msg_python(msg_class):
                        # Get C++ equivalent
                        cpp_msg_type, cpp_msg_class = _resolve_message_type(msg_class)
                        enable_kwargs_for_cpp_msg(cpp_msg_class)
                        add_python_msg_sugar_to_cpp_msg(cpp_msg_class, msg_class)
                        
                        # Replace the class in the module
                        setattr(module, msg_name, cpp_msg_class)
                        print(f"Auto-converted {module.__name__}.{msg_name} to C++")
                except Exception as e:
                    print(f"Failed to convert {msg_name}: {e}")
    
    # Install the import hook if not already installed
    if not any(isinstance(finder, ROSMessageConverter) for finder in sys.meta_path):
        sys.meta_path.insert(0, ROSMessageConverter())
        print("Automatic C++ message conversion enabled!")

if __name__ == "__main__":
    # Example 1: Manual approach (what you had before)
    print("=== Manual Approach ===")
    rclcpp = bringup_rclcpp()
    # from std_msgs.msg import String
    cppyy.include("std_msgs/msg/string.hpp")
    CppString = cppyy.gbl.std_msgs.msg.String
    enable_kwargs_for_cpp_msg(CppString)
    
    node = RclcppyyNode("rclcppyy_node")
    publisher = node.create_publisher(CppString, "test_topic", 10)
    msg = CppString(data="Hello from manual approach!")
    print(f"Manual: Created message with data='{msg.data}'")
    
    # # Example 2: Post-import global replacement (Approach 1 - Recommended)
    # print("\n=== Post-Import Global Replacement ===")
    # from std_msgs.msg import String as PyString
    # from geometry_msgs.msg import TwistStamped
    # from rcl_interfaces.msg import ParameterValue
    
    # print("Before conversion:", type(PyString))
    # convert_python_msgs_to_cpp()  # This will convert all Python messages in globals()
    # print("After conversion:", type(PyString))
    
    # # Now PyString is actually a C++ class with keyword support!
    # cpp_msg = PyString(data="Hello from auto-converted C++!")
    # print(f"Auto-converted: Created message with data='{cpp_msg.data}'")
    
    # Example 3: Automatic import hook (Approach 2 - Advanced)
    print("\n=== Automatic Import Hook ===")
    setup_automatic_cpp_conversion()  # Enable the import hook

    from sensor_msgs.msg import Image, PointCloud2, JointState
    print(f"Image: {Image}")
    print(f"PointCloud2: {PointCloud2}")
    print(f"JointState: {JointState}")
    print(f"Image(): {Image()}")
    print(f"PointCloud2(): {PointCloud2()}")
    print(f"JointState(): {JointState()}")

    
    # Any future imports will be automatically converted
    # (This would work for fresh imports in a new Python session)
    
    timer = node.create_timer(1.0, lambda: publisher.publish(msg))
    
    print("\nStarting ROS spinning...")
    node._rclcpp.spin(node._rclcpp_node)
    rclpy.shutdown()