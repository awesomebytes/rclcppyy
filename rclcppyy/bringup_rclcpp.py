import cppyy
import os
import re
from functools import lru_cache
from typing import Any, List, Optional, Set, Dict
from ament_index_python.packages import get_package_prefix, get_packages_with_prefixes

RCLCPP_BRINGUP_DONE = False

def get_ros2_include_path() -> str:
    ROS2_INCLUDE_PATH = get_package_prefix("rclcpp") + "/include"
    return ROS2_INCLUDE_PATH

def add_ros2_include_path() -> bool:
    ROS2_INCLUDE_PATH = get_ros2_include_path()
    return cppyy.add_include_path(ROS2_INCLUDE_PATH)

def add_ros2_include_paths() -> bool:
    ROS2_INCLUDE_PATH = get_ros2_include_path()
    ROS2_LIB_PATH = get_package_prefix("rclcpp") + "/lib"
    # All of these are needed for rclcpp to work

    # Instead of hardcoding the list, just get all the available packages
    pkgs_with_prefixes = get_packages_with_prefixes()
    for pkg, prefix in pkgs_with_prefixes.items():
        include_path = os.path.join(prefix, "include", pkg)
        # The path may not exist if its a Python package (or simply does not have headers)
        if os.path.exists(include_path):
            cppyy.add_include_path(include_path)

    return True


def get_ros2_lib_path() -> str:
    return os.path.join(get_package_prefix("rclcpp"), "lib")


# Track what we've already handed to cppyy so bringup / message resolution stay
# idempotent.
_ROS_CORE_LIBS_LOADED = False
_LOADED_MSG_TYPESUPPORT: Set[str] = set()


def ensure_ros_libraries_loaded() -> None:
    """
    Make cppyy resolve rclcpp/rcl symbols without relying on LD_LIBRARY_PATH.

    cppyy discovers which shared library owns a symbol by scanning
    LD_LIBRARY_PATH at call-time resolution. The pixi workspace sets
    LD_LIBRARY_PATH=$CONDA_PREFIX/lib in activation, but an installed conda
    package has no such activation, so calls like rclcpp::init would fail with
    "failed to resolve function". Instead, add the ROS lib dir to cppyy's own
    library search path and eagerly load librclcpp (whose RPATH pulls in
    rcl/rmw/rcutils/... and the typesupport rclcpp itself needs). Per-message
    typesupport is loaded on demand in _resolve_message_type. Harmless (and
    idempotent) when LD_LIBRARY_PATH is already set.
    """
    global _ROS_CORE_LIBS_LOADED
    if _ROS_CORE_LIBS_LOADED:
        return
    cppyy.add_library_path(get_ros2_lib_path())
    cppyy.load_library("librclcpp.so")
    _ROS_CORE_LIBS_LOADED = True


def load_ros_library(libname: str) -> None:
    """
    Load a ROS shared library by name (e.g. "librosbag2_storage.so") via cppyy,
    so its symbols resolve without LD_LIBRARY_PATH. Ensures the ROS lib dir is on
    cppyy's search path first. No-op if the library is missing.
    """
    ensure_ros_libraries_loaded()
    try:
        cppyy.load_library(libname)
    except Exception:
        pass


def _load_message_typesupport(package: str) -> None:
    """
    Load a message package's C++ typesupport library so cppyy can resolve its
    rosidl_typesupport_cpp::get_message_type_support_handle<...> symbol without
    LD_LIBRARY_PATH. No-op if already handled or if the package ships no such
    library (e.g. pure-Python message packages).
    """
    if package in _LOADED_MSG_TYPESUPPORT:
        return
    _LOADED_MSG_TYPESUPPORT.add(package)
    try:
        cppyy.load_library(f"lib{package}__rosidl_typesupport_cpp.so")
    except Exception:
        pass


def force_symbol_discovery(namespace: Any, known_symbols: Set[str] = None) -> Dict[str, Any]:
    """
    Force discovery of all symbols in a cppyy namespace by accessing them.
    Returns a dictionary of discovered symbols and their types.
    """
    if known_symbols is None:
        known_symbols = set()
    
    discovered = {}
    current_symbols = set(dir(namespace))
    new_symbols = current_symbols - known_symbols
    
    for symbol_name in new_symbols:
        if symbol_name.startswith('__'):
            continue
            
        try:
            symbol = getattr(namespace, symbol_name)
            symbol_type = type(symbol).__name__
            
            # Try to determine what kind of symbol this is
            if hasattr(symbol, '__call__'):
                if 'function' in symbol_type.lower():
                    discovered[symbol_name] = {'type': 'function', 'object': symbol}
                else:
                    discovered[symbol_name] = {'type': 'callable', 'object': symbol}
            elif 'type' in symbol_type.lower() or 'class' in symbol_type.lower():
                discovered[symbol_name] = {'type': 'class', 'object': symbol}
                
                # For classes, also explore their methods
                try:
                    class_methods = dir(symbol)
                    discovered[symbol_name]['methods'] = [m for m in class_methods if not m.startswith('__')]
                except:
                    pass
            elif symbol_type == 'CPPScope':
                discovered[symbol_name] = {'type': 'namespace', 'object': symbol}
            else:
                discovered[symbol_name] = {'type': 'other', 'object': symbol, 'python_type': symbol_type}
                
        except Exception as e:
            discovered[symbol_name] = {'type': 'error', 'error': str(e)}
    
    return discovered


def recursive_symbol_discovery(namespace: Any, namespace_name: str = "", max_depth: int = 3, current_depth: int = 0) -> Dict[str, Dict]:
    """
    Recursively discover symbols in nested namespaces.
    """
    if current_depth >= max_depth:
        return {}
    
    # print(f"{'  ' * current_depth}Exploring namespace: {namespace_name}")
    
    # Get initial symbols
    initial_symbols = set(dir(namespace))
    discovered = force_symbol_discovery(namespace, set())
    
    # Check if accessing symbols revealed new ones
    iterations = 0
    while iterations < 5:  # Limit iterations to prevent infinite loops
        current_symbols = set(dir(namespace))
        if current_symbols == initial_symbols:
            break
            
        new_discovered = force_symbol_discovery(namespace, initial_symbols)
        discovered.update(new_discovered)
        initial_symbols = current_symbols
        iterations += 1
    
    # Recursively explore namespaces
    for symbol_name, symbol_info in discovered.items():
        if symbol_info.get('type') == 'namespace':
            sub_namespace_name = f"{namespace_name}.{symbol_name}" if namespace_name else symbol_name
            try:
                sub_discovered = recursive_symbol_discovery(
                    symbol_info['object'], 
                    sub_namespace_name, 
                    max_depth, 
                    current_depth + 1
                )
                discovered[symbol_name]['sub_symbols'] = sub_discovered
            except Exception as e:
                discovered[symbol_name]['sub_error'] = str(e)
    
    return discovered


def explore_known_rclcpp_classes(verbose: bool = False):
    """
    Force discovery of commonly known rclcpp classes to trigger their loading.
    """
    known_classes = [
        'Node', 'NodeOptions', 'Publisher', 'Subscription', 'Client', 'Service',
        'QoS', 'KeepAll', 'KeepLast', 'Clock', 'Time', 'Duration', 'Rate',
        'Logger', 'Parameter', 'ParameterValue', 'CallbackGroup', 'Executor',
        'Context'
    ]
    # Not found:
    # 'SingleThreadedExecutor', 'MultiThreadedExecutor', 'Timer'
    
    if verbose:
        print("Attempting to access known rclcpp classes...")
    discovered_classes = []
    
    for class_name in known_classes:
        try:
            cls = getattr(cppyy.gbl.rclcpp, class_name)
            discovered_classes.append(class_name)
            if verbose:
                print(f"  ✓ Found: {class_name}")
        except AttributeError:
            if verbose:
                print(f"  ✗ Not found: {class_name}")
        except Exception as e:
            print(f"  ⚠ Error accessing {class_name}: {e}")
    
    return discovered_classes

@staticmethod
def _is_msg_cpp(message_type):
    """
    Check if the message type is a C++ cppyy message class.
    Fast check using unique cppyy attributes.
    """
    # Fast check: cppyy objects have __smartptr__ attribute that Python messages don't
    return hasattr(message_type, '__smartptr__')

@staticmethod
def _is_msg_python(message_type):
    """
    Check if the message type is a Python ROS message class.
    Fast check using __slots__ attribute that cppyy messages don't have.
    """
    # Fast check: Python ROS messages have __slots__ attribute that cppyy messages don't
    return hasattr(message_type, '__slots__')

# @lru_cache(maxsize=1000)
@staticmethod
def _resolve_message_type(message_type):
    """
    Resolve message type to C++ type string and cppyy type object.
    
    Args:
        message_type: Can be:
            - Python ROS message class (e.g., std_msgs.msg.String)
            - C++ cppyy message class (e.g., cppyy.gbl.std_msgs.msg.String)
    
    Returns:
        tuple: (cpp_type_string, cppyy_type_object)
    """
    # Case 1: C++ cppyy message class (check this BEFORE Python messages)
    if _is_msg_cpp(message_type):
        # Extract type information from cppyy type
        module_parts = message_type.__module__.split('.')
        
        # Handle cppyy.gbl.package_msgs.msg structure (4 parts)
        if len(module_parts) >= 4:  # ['cppyy', 'gbl', 'package_msgs', 'msg']
            package = module_parts[2]  # e.g., 'std_msgs'
            # Get message name from class name, removing template suffixes
            class_name = message_type.__name__
            # Remove template parameters like '_<std::allocator<void>>'
            msg_name = re.sub(r'_<.*?>$', '', class_name)
            cpp_type_str = f"{package}::msg::{msg_name}"

            _load_message_typesupport(package)
            # Since the cppyy type is already available and the header is included, just return it
            return cpp_type_str, message_type
        else:
            raise ValueError(f"Cannot parse C++ cppyy message type: {message_type}")
    
    # Case 2: Python ROS message class (e.g., std_msgs.msg.String)
    elif _is_msg_python(message_type):
        # Extract package and message name from Python ROS message type
        module_parts = message_type.__module__.split('.')
        if len(module_parts) >= 2:
            package = module_parts[0]  # e.g., 'std_msgs'

            # Get message name from class name, removing 'Metaclass_' prefix if present
            class_name = message_type.__class__.__name__
            if class_name.startswith('Metaclass_'):
                msg_name = class_name[10:]  # Remove 'Metaclass_' prefix
            else:
                msg_name = class_name
            
            # Convert to lowercase for header file
            msg_name_lower = msg_name.lower()
            
            # Build C++ type string and include header
            cpp_type_str = f"{package}::msg::{msg_name}"
            # module_parts looks like:
            # std_msgs.msg._multi_array_layout or rcl_interfaces.msg._parameter_event
            hpp_file_name = f"{module_parts[2][1:]}.hpp"
            header_path = f"{package}/msg/{hpp_file_name}"
            # print(f"header_path: {header_path}, package: {package}, msg_name: {msg_name}, msg_name_lower: {msg_name_lower}, cpp_type_str: {cpp_type_str}, module_parts: {module_parts}, hpp_file_name: {hpp_file_name}")
            cppyy.add_include_path(os.path.join(get_package_prefix(package), "include", package))
            cppyy.include(header_path)
            _load_message_typesupport(package)
            # print(f"included header: {header_path}")

            # Get the C++ type
            cppyy_type = getattr(getattr(getattr(cppyy.gbl, package), 'msg'), msg_name)
            return cpp_type_str, cppyy_type
        else:
            raise ValueError(f"Cannot parse Python ROS message type: {message_type}")
    
    else:
        raise ValueError(f"Unsupported message type: {message_type} (type: {type(message_type)})")


def adapt_rclcpp_to_python(rclcpp: Any):
    """
    Adapt the rclcpp namespace to be more Pythonic.
    """
    rclcpp_init = rclcpp.init
    def rclpy_init(args: Optional[List[str]] = None, *, context=None) -> None:
        if args is None:
            args = []
        rclcpp_init(len(args), args)

    def init_wrapper(*args, **kwargs):
        # Handle both rclpy and rclcpp style calls
        if len(args) == 2 and isinstance(args[0], int):
            # rclcpp style: init(argc, argv)
            return rclcpp_init(*args)
        else:
            # rclpy style: init(args=None, *, context=None)
            return rclpy_init(*args, **kwargs)

    rclcpp.init = init_wrapper

    # rclcpp::spin needs a shared_ptr-managed node. cppyy can hand it one for a
    # directly-created rclcpp.Node, but not for a Python subclass of rclcpp.Node
    # (as the rclpy tutorials write them). In that case fall back to the node's
    # base interface, which spin also accepts.
    rclcpp_spin = rclcpp.spin
    def spin_wrapper(node, *args, **kwargs):
        try:
            return rclcpp_spin(node, *args, **kwargs)
        except TypeError:
            if hasattr(node, "get_node_base_interface"):
                return rclcpp_spin(node.get_node_base_interface(), *args, **kwargs)
            raise

    rclcpp.spin = spin_wrapper

def _keep_alive(node: Any, *objs: Any) -> None:
    """
    Keep Python objects referenced for as long as the node lives.

    cppyy's std::function wrappers do not reliably hold a strong reference to the
    underlying Python callable, so without this the callback can be garbage
    collected and firing it later raises "callable was deleted".
    """
    store = getattr(node, "_rclcppyy_kept_alive", None)
    if store is None:
        store = []
        node._rclcppyy_kept_alive = store
    store.extend(objs)


def _wrap_publish_for_python_msgs(publisher: Any) -> None:
    """
    Wrap a publisher's publish() so that, if handed an rclpy (Python) message,
    it is converted to the equivalent rclcpp (C++) message before publishing.
    C++ messages are published directly with no conversion overhead.
    """
    original_publish = publisher.publish

    def publish_wrapper(msg):
        if _is_msg_cpp(msg):
            return original_publish(msg)
        # Convert the Python message to its C++ equivalent (top-level fields).
        # Nested messages/arrays need recursive handling (see
        # RclcppyyNode._convert_msg_to_cpp); this simple path covers flat messages.
        _, cppyy_type = _resolve_message_type(msg)
        cpp_msg = cppyy_type()
        for field_name in msg.get_fields_and_field_types():
            setattr(cpp_msg, field_name, getattr(msg, field_name))
        return original_publish(cpp_msg)

    publisher.publish = publish_wrapper


# The native templated methods are stashed under these names so the bound
# handlers can invoke them through the node instance (cppyy binds `this` for us,
# which subscription template-argument deduction requires).
_ORIG_CREATE_PUBLISHER = "_rclcppyy_orig_create_publisher"
_ORIG_CREATE_SUBSCRIPTION = "_rclcppyy_orig_create_subscription"


class _BoundCreatePublisher:
    """Per-node handler installed by the create_publisher descriptor."""

    def __init__(self, node: Any):
        self._node = node

    def _orig(self):
        # Native templated create_publisher, bound to this node.
        return getattr(self._node, _ORIG_CREATE_PUBLISHER)

    def __getitem__(self, msg_type):
        # Native rclcpp template syntax: node.create_publisher[MsgT](topic, qos, ...)
        return self._orig()[msg_type]

    def __call__(self, msg_type, topic, qos, *args, **kwargs):
        # rclpy style: node.create_publisher(MsgType, topic, qos)
        _, cppyy_type = _resolve_message_type(msg_type)
        publisher = self._orig()[cppyy_type](topic, qos, *args, **kwargs)
        # Only the rclpy-style path may be handed Python messages, so this is
        # where we install the (opt-in) publish conversion wrapper.
        _wrap_publish_for_python_msgs(publisher)
        return publisher


class _BoundCreateSubscription:
    """Per-node handler installed by the create_subscription descriptor."""

    def __init__(self, node: Any):
        self._node = node

    def _orig(self):
        # Native templated create_subscription, bound to this node.
        return getattr(self._node, _ORIG_CREATE_SUBSCRIPTION)

    def __getitem__(self, msg_type):
        # Native rclcpp template syntax: node.create_subscription[MsgT](topic, qos, cb)
        return self._orig()[msg_type]

    def __call__(self, msg_type, topic, callback, qos, *args, **kwargs):
        # rclpy style: node.create_subscription(MsgType, topic, callback, qos)
        cpp_type_str, cppyy_type = _resolve_message_type(msg_type)
        # cppyy delivers the C++ message proxy straight to the Python callback and
        # manages the GIL, so no manual pointer rebinding is needed.
        cpp_callback = cppyy.gbl.std.function[
            f"void(std::shared_ptr<const {cpp_type_str}>)"
        ](callback)
        subscription = self._orig()[cppyy_type](topic, qos, cpp_callback, *args, **kwargs)
        _keep_alive(self._node, callback, cpp_callback)
        return subscription


class _NodeMethodDescriptor:
    """
    Descriptor placed on rclcpp.Node that hands out a per-instance handler. The
    handler preserves the native ``method[MsgT](...)`` template syntax while also
    accepting rclpy-style positional calls (see the bound handlers).
    """

    def __init__(self, bound_cls):
        self._bound_cls = bound_cls

    def __get__(self, instance, owner=None):
        if instance is None:
            return self
        return self._bound_cls(instance)


def adapt_node_pub_sub_to_python(rclcpp: Any):
    """
    Make rclcpp.Node.create_publisher / create_subscription accept the rclpy
    calling convention in addition to the native rclcpp template syntax, so the
    same node object works with either API:

        # rclpy style (Python message classes auto-resolved to C++):
        pub = node.create_publisher(String, 'topic', 10)
        sub = node.create_subscription(String, 'topic', cb, 10)

        # native rclcpp style (unchanged, still zero-overhead):
        pub = node.create_publisher[cppyy.gbl.std_msgs.msg.String]('topic', 10)

    Idempotent: applying it more than once is a no-op.
    """
    if getattr(rclcpp.Node, "_rclcppyy_pubsub_adapted", False):
        return
    setattr(rclcpp.Node, _ORIG_CREATE_PUBLISHER, rclcpp.Node.create_publisher)
    setattr(rclcpp.Node, _ORIG_CREATE_SUBSCRIPTION, rclcpp.Node.create_subscription)
    rclcpp.Node.create_publisher = _NodeMethodDescriptor(_BoundCreatePublisher)
    rclcpp.Node.create_subscription = _NodeMethodDescriptor(_BoundCreateSubscription)
    rclcpp.Node._rclcppyy_pubsub_adapted = True


def adapt_node_timer_to_python(rclcpp: Any):
    """
    Let timers be created the rclpy way on a plain rclcpp.Node:

        self.timer = node.create_timer(period_seconds, callback)

    backed by rclcpp's create_wall_timer plus a std::function wrapping the Python
    callable. Idempotent.
    """
    if getattr(rclcpp.Node, "_rclcppyy_timer_adapted", False):
        return

    def create_timer_wrapper(self, timer_period_sec, callback, *args, **kwargs):
        period_ns = int(timer_period_sec * 1e9)
        cpp_callback = cppyy.gbl.std.function["void()"](callback)
        timer = self.create_wall_timer(
            cppyy.gbl.std.chrono.nanoseconds(period_ns), cpp_callback)
        _keep_alive(self, callback, cpp_callback)
        return timer

    rclcpp.Node.create_timer = create_timer_wrapper
    rclcpp.Node._rclcppyy_timer_adapted = True


def adapt_node_lifecycle_to_python(rclcpp: Any):
    """
    Provide the rclpy Node lifecycle method destroy_node() on a plain rclcpp.Node.

    rclcpp nodes clean up via RAII when they (and the entities holding their
    shared_ptrs) are released, so there is no explicit destroy step; this shim
    releases the Python callbacks we pinned and is otherwise a no-op, letting
    unmodified rclpy tutorial code (minimal_publisher.destroy_node()) run.
    Idempotent.
    """
    if getattr(rclcpp.Node, "_rclcppyy_lifecycle_adapted", False):
        return

    def destroy_node_wrapper(self, *args, **kwargs):
        store = getattr(self, "_rclcppyy_kept_alive", None)
        if store is not None:
            store.clear()

    rclcpp.Node.destroy_node = destroy_node_wrapper
    rclcpp.Node._rclcppyy_lifecycle_adapted = True


def adapt_get_logger_to_python(rclcpp: Any):
    """
    Adapt the rclcpp get_logger method so it can be called the python way.
    """
    original_get_logger = rclcpp.Node.get_logger
    def get_logger_wrapper(this_self, *args, **kwargs):
        # we need to return a logger that can do all the levels
        # debug, info, warn, error, fatal
        # Given the logger and logger levels in c++ are macros, here, we will just go ahead and use the
        # rclpy logger api to get the logger and then use the logger in the python way
        from rclpy.node import get_logger
        return get_logger(this_self.get_name())

    rclcpp.Node.get_logger = get_logger_wrapper


def bringup_rclcpp():
    """
    Bring up the rclcpp library.
    This function should be called before using any rclcpp classes.
    It adds the necessary include paths and loads the necessary libraries.
    Returns the rclcpp namespace.
    """
    global RCLCPP_BRINGUP_DONE
    if RCLCPP_BRINGUP_DONE:
        return cppyy.gbl.rclcpp
    
    add_ros2_include_paths()
    ensure_ros_libraries_loaded()
    print("Including rclcpp.hpp with cppyy, this may take a few seconds... WIP to remove this slowdown")
    cppyy.include("rclcpp/rclcpp.hpp")
    cppyy.include("rcl_interfaces/msg/parameter_event.hpp")
    cppyy.include("chrono")
    cppyy.include("functional")
    print("Done bringing up rclcpp.")
    
    explore_known_rclcpp_classes()
    recursive_symbol_discovery(cppyy.gbl.rclcpp, "rclcpp", max_depth=5)
    adapt_rclcpp_to_python(cppyy.gbl.rclcpp)
    adapt_node_pub_sub_to_python(cppyy.gbl.rclcpp)
    adapt_node_timer_to_python(cppyy.gbl.rclcpp)
    adapt_node_lifecycle_to_python(cppyy.gbl.rclcpp)
    adapt_get_logger_to_python(cppyy.gbl.rclcpp)

    RCLCPP_BRINGUP_DONE = True
    # TODO: Add rclcpp to the global namespace or another user-friendly way of givin access
    return cppyy.gbl.rclcpp
