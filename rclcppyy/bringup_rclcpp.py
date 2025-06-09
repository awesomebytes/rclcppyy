import cppyy
import os
import re
from typing import Any, List, Optional, Set, Dict
from ament_index_python.packages import get_package_prefix

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
    minimal_rclcpp_includes = [
        'rclcpp',
        'std_msgs', 
        'rcl_interfaces',
        'rosidl_runtime_cpp',
        'rosidl_runtime_c',
        'service_msgs',
        'builtin_interfaces',
        'rosidl_typesupport_interface',
        'rcutils',
        'rcl',
        'rmw',
        'rcpputils',
        'rcl_yaml_param_parser',
        'type_description_interfaces',
        'rosidl_dynamic_typesupport',
        'tracetools',
        'libstatistics_collector',
        'statistics_msgs',
        'rosidl_typesupport_introspection_cpp'
    ]
    for include in minimal_rclcpp_includes:
        cppyy.add_include_path(os.path.join(ROS2_INCLUDE_PATH, include))
    
    for pkg in minimal_rclcpp_includes:
        try:
            cppyy.load_library(os.path.join(ROS2_LIB_PATH, f"lib{pkg}.so"))
        except Exception as e:
            # Some packages are header-only, so we can ignore the error
            # TODO: double-check if this is needed at all
            # I don't think it is, but it takes no time to load them
            pass
    return True

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
        'Node', 'NodeOptions', 'Publisher', 'Subscription', 'Timer', 'Client', 'Service',
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
    """
    if hasattr(message_type, '__module__') and 'cppyy.gbl' in str(message_type.__module__):
        return True
    return False

@staticmethod
def _is_msg_python(message_type):
    """
    Check if the message type is a Python ROS message class.
    """
    if hasattr(message_type, '__module__') and 'msg' in str(message_type.__module__):
        return True
    return False

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
            header_path = f"{package}/msg/{msg_name_lower}.hpp"
            
            cppyy.include(header_path)
            
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

def adapt_node_pub_sub_to_python(rclcpp: Any):
    """
    Adapt the rclcpp publisher and subscriber classes so they automatically
    adapt to the message types and when someone calls either the rclpy Node api
    self.publisher_ = self.create_publisher(String, 'topic', 10)
    or the rclcpp Node api
    self.publisher = self.node.create_publisher[cppyy.gbl.std_msgs.msg.String](
        "perf_topic_pythonic", 10)
    It just works.
    """
    # Store original methods
    original_create_publisher = rclcpp.Node.create_publisher
    original_create_subscription = rclcpp.Node.create_subscription

    def create_publisher_wrapper(self, *args, **kwargs):
        # Handle rclpy style: create_publisher(msg_type, topic, qos)
        if len(args) == 3:
            msg_type, topic, qos = args
            cpp_type_str, cppyy_type = _resolve_message_type(msg_type)
            ret_publisher = original_create_publisher[cppyy_type](self, topic, qos)
        # Handle rclcpp style: create_publisher[msg_type](topic, qos)
        else:
            ret_publisher = original_create_publisher(*args, **kwargs)
        
        # We need to make sure that the Publisher.publish, if its given a Python message, we convert it to a C++ message
        # and then call the original publish method.
        original_publish = ret_publisher.publish
        def publish_wrapper(msg):
            if _is_msg_cpp(msg):
                return original_publish(msg)
            else:
                cpp_type_str, cppyy_type = _resolve_message_type(msg)
                cpp_msg = cppyy_type()

                # Note: this is not very efficient, but it works.
                # We should instead monkeypatch the imported messages to convert them to cpp from the start
                # That way we will work directly with cpp objects and we avoid this overhead
                for dict_key, dict_val in msg.get_fields_and_field_types().items():
                    setattr(cpp_msg, dict_key, getattr(msg, dict_key))

            return original_publish(cpp_msg)
        ret_publisher.publish = publish_wrapper
        return ret_publisher

    def create_subscription_wrapper(self, *args, **kwargs):
        # Handle rclpy style: create_subscription(msg_type, topic, callback, qos)
        if len(args) == 4:
            msg_type, topic, callback, qos = args
            cpp_type_str, cppyy_type = _resolve_message_type(msg_type)
            
            # Create a C++ callback wrapper that will call the Python callback
            cppyy.cppdef("""
                #include <Python.h>
                #include <functional>
                
                template<typename T>
                static std::function<void(const typename T::SharedPtr)> 
                create_subscription_callback_wrapper(PyObject* callback) {
                    return [callback](const typename T::SharedPtr msg) {
                        if (callback && PyCallable_Check(callback)) {
                            PyObject_CallFunction(callback, "O", msg);
                        }
                    };
                }
            """)
            
            cpp_callback = getattr(cppyy.gbl, f"create_subscription_callback_wrapper<{cpp_type_str}>")(callback)
            return original_create_subscription[cppyy_type](self, topic, qos, cpp_callback)
        # Handle rclcpp style: create_subscription[msg_type](topic, qos, callback)
        else:
            return original_create_subscription(*args, **kwargs)

    # Replace the original methods with our wrappers
    rclcpp.Node.create_publisher = create_publisher_wrapper
    rclcpp.Node.create_subscription = create_subscription_wrapper

def adapt_timer_to_python(rclcpp: Any):
    """
    Adapt the rclcpp timer class so it can be called the python way.
    # rclpy way
    self.timer = self.create_timer(1, self.timer_callback)
    def timer_callback(self):
        print("timer_callback")
    """
    # Define the C++ callback wrapper template only once
    cppyy.cppdef("""
        #include <Python.h>
        #include <functional>
        
        static std::function<void()> create_timer_callback(PyObject* self) {
            return [self]() {
                if (self && PyObject_HasAttrString(self, "timer_callback")) {
                    PyObject_CallMethod(self, "timer_callback", nullptr);
                }
            };
        }
    """)

    def create_timer_wrapper(self, period_seconds, callback):
        """
        Python-style timer creation that matches rclpy's API:
        create_timer(period_seconds: float, callback: Callable)
        """
        # Convert period to nanoseconds for rclcpp  
        period_ns = int(period_seconds * 1e9)
        
        # Store the callback directly on the node object
        # This is simpler than the unique method name approach
        self.timer_callback = callback
        
        # Create the C++ callback wrapper
        cpp_callback = cppyy.gbl.create_timer_callback(self)
        
        # Create the timer exactly like the working benchmark does
        # Directly call create_wall_timer on the node
        wall_timer =self.create_wall_timer(
            cppyy.gbl.std.chrono.nanoseconds(period_ns),
            cpp_callback
        )
        return wall_timer

    # Add create_timer method to Node class (it might not exist in rclcpp)
    rclcpp.Node.create_timer = create_timer_wrapper

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
    print("Including rclcpp.hpp with cppyy, this may take a few seconds... WIP to remove this slowdown")
    cppyy.include("rclcpp/rclcpp.hpp")
    cppyy.include("chrono")
    cppyy.include("functional")
    print("Done bringing up rclcpp.")
    explore_known_rclcpp_classes()
    recursive_symbol_discovery(cppyy.gbl.rclcpp, "rclcpp", max_depth=3)
    adapt_rclcpp_to_python(cppyy.gbl.rclcpp)
    adapt_node_pub_sub_to_python(cppyy.gbl.rclcpp)
    adapt_timer_to_python(cppyy.gbl.rclcpp)
    adapt_get_logger_to_python(cppyy.gbl.rclcpp)

    RCLCPP_BRINGUP_DONE = True
    # TODO: Add rclcpp to the global namespace or another user-friendly way of givin access
    return cppyy.gbl.rclcpp
