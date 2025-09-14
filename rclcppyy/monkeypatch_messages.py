"""
This file implements monkeypatching of ROS2 imported messages
so they are created as rclcpp messages instead of rclpy messages.
This avoids message conversion overhead when publishing/subscribing.
"""

import sys
import inspect
import importlib.util
from importlib.abc import Loader, MetaPathFinder
from importlib.machinery import ModuleSpec
from rclcppyy.bringup_rclcpp import _resolve_message_type, _is_msg_python

def enable_kwargs_for_cpp_msg(msg_class):
    """Enable keyword arguments for C++ message constructors"""
    original_init = msg_class.__init__
    def new_init(self, *args, **kwargs):
        # Call the original constructor with default arguments
        if args:
            original_init(self, *args)
        else:
            original_init(self)
        
        # Set any keyword arguments as attributes
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
            else:
                raise AttributeError(f"'{self.__class__.__name__}' object has no attribute '{key}'")

    msg_class.__init__ = new_init

def add_python_compatibility(cpp_msg_class, py_msg_class):
    """Add Python message methods to C++ message class for compatibility"""
    print(f"Adding Python compatibility to {cpp_msg_class} and {py_msg_class}")
    print(f"py_msg_class.__dict__: {py_msg_class.__dict__}")
    if hasattr(py_msg_class, '__repr__'):
        cpp_msg_class.__repr__ = py_msg_class.__repr__

    # Get the attrs from py_msg_class that are not in cpp_msg_class already and that do not start with __
    py_msg_class_attrs = [attr for attr in dir(py_msg_class) if not attr.startswith("__")]
    cpp_msg_class_attrs = [attr for attr in dir(cpp_msg_class) if not attr.startswith("__")]
    attrs_to_add = [attr for attr in py_msg_class_attrs if attr not in cpp_msg_class_attrs]

    for attr_name in attrs_to_add:
        setattr(cpp_msg_class, attr_name, getattr(py_msg_class, attr_name))

    # Special case for service messages, they depend on their __class__ having the following attributes
    if hasattr(py_msg_class.__class__,"_TYPE_SUPPORT"):
        cpp_msg_class.__class__._TYPE_SUPPORT = py_msg_class.__class__._TYPE_SUPPORT
    if hasattr(py_msg_class.__class__, "__import_type_support__"):
        cpp_msg_class.__class__.__import_type_support__ = py_msg_class.__class__.__import_type_support__

class ROSMessageFinder(MetaPathFinder):
    """Import hook to automatically convert ROS messages to C++ equivalents"""
    
    def __init__(self):
        self._importing = set()
        self._original_loaders = {}
    
    def find_spec(self, fullname, path, target=None):
        # Only intercept ROS message type imports
        if '.msg._' in fullname and fullname not in self._importing:
            self._importing.add(fullname)
            try:
                spec = importlib.util.find_spec(fullname)
            finally:
                self._importing.remove(fullname)
            
            if spec is not None and spec.loader is not None:
                self._original_loaders[fullname] = spec.loader
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
        return self._original_loader.create_module(spec)
    
    def exec_module(self, module):
        # Execute with original loader first
        self._original_loader.exec_module(module)
        
        # Get the message name from the module
        msg_names = [attr for attr in dir(module) if attr.startswith("Metaclass_")]
        for metaclass_name in msg_names:
            msg_name = metaclass_name.replace("Metaclass_", "")
            
            # Only convert the actual message class
            if hasattr(module, msg_name):
                msg_class = getattr(module, msg_name)
                try:
                    if _is_msg_python(msg_class):
                        # Get C++ equivalent
                        _, cpp_msg_class = _resolve_message_type(msg_class)
                        enable_kwargs_for_cpp_msg(cpp_msg_class)
                        add_python_compatibility(cpp_msg_class, msg_class)
                        
                        # Replace the class in the module
                        setattr(module, msg_name, cpp_msg_class)
                except Exception as e:
                    print(f"Failed to convert {msg_name}: {e}")

def convert_already_imported_python_msgs_to_cpp(target_globals=None):
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
            add_python_compatibility(cpp_msg_class, py_msg_class)
            
            # Replace in globals
            target_globals[name] = cpp_msg_class
            print(f"Converted {name}: {py_msg_class} -> {cpp_msg_class}")
            
        except Exception as e:
            print(f"Failed to convert {name}: {e}")
            # Keep the original Python version if conversion fails
            continue


def install_ros_message_hook():
    """Install the import hook if not already installed"""
    if not any(isinstance(finder, ROSMessageFinder) for finder in sys.meta_path):
        sys.meta_path.insert(0, ROSMessageFinder())
        return True
    return False