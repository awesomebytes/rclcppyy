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
            
            # Use the generic callback template
            # Need to use the template with bracket syntax for cppyy
            make_callback = cppyy.gbl.make_py_sub_callback[cpp_type_str]
            cpp_callback = make_callback(callback)
            return original_create_subscription[cppyy_type](self, topic, qos, cpp_callback)
        # Handle rclcpp style: create_subscription[msg_type](topic, qos, callback)
        else:
            return original_create_subscription(*args, **kwargs)

    # Replace the original methods with our wrappers
    rclcpp.Node.create_publisher = create_publisher_wrapper
    rclcpp.Node.create_subscription = create_subscription_wrapper

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
    cppyy.include("rcl_interfaces/msg/parameter_event.hpp")
    cppyy.include("chrono")
    cppyy.include("functional")
    print("Done bringing up rclcpp.")
    
    explore_known_rclcpp_classes()
    recursive_symbol_discovery(cppyy.gbl.rclcpp, "rclcpp", max_depth=5)
    adapt_rclcpp_to_python(cppyy.gbl.rclcpp)
    # adapt_node_pub_sub_to_python(cppyy.gbl.rclcpp)
    adapt_get_logger_to_python(cppyy.gbl.rclcpp)

    RCLCPP_BRINGUP_DONE = True
    # TODO: Add rclcpp to the global namespace or another user-friendly way of givin access
    return cppyy.gbl.rclcpp
