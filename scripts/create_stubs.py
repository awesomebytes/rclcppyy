#!/usr/bin/env python3
"""
Dynamic exploration of all rclcpp symbols using cppyy.
This script forces discovery of all classes, functions, and other elements.
"""
import cppyy
import inspect
import pydoc
import re
from pathlib import Path
from typing import Set, Dict, List, Any, Optional, Tuple
from rclcppyy.bringup_rclcpp import bringup_rclcpp

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
    
    print(f"{'  ' * current_depth}Exploring namespace: {namespace_name}")
    
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


def explore_known_rclcpp_classes():
    """
    Force discovery of commonly known rclcpp classes to trigger their loading.
    """
    known_classes = [
        'Node', 'NodeOptions', 'Publisher', 'Subscription', 'Timer', 'Client', 'Service',
        'QoS', 'KeepAll', 'KeepLast', 'Clock', 'Time', 'Duration', 'Rate',
        'Logger', 'Parameter', 'ParameterValue', 'CallbackGroup', 'Executor',
        'SingleThreadedExecutor', 'MultiThreadedExecutor', 'Context'
    ]
    
    print("Attempting to access known rclcpp classes...")
    discovered_classes = []
    
    for class_name in known_classes:
        try:
            cls = getattr(cppyy.gbl.rclcpp, class_name)
            discovered_classes.append(class_name)
            print(f"  ✓ Found: {class_name}")
        except AttributeError:
            print(f"  ✗ Not found: {class_name}")
        except Exception as e:
            print(f"  ⚠ Error accessing {class_name}: {e}")
    
    return discovered_classes


def print_symbol_summary(discovered: Dict[str, Dict], indent: int = 0):
    """
    Print a summary of discovered symbols organized by type.
    """
    indent_str = "  " * indent
    
    by_type = {}
    for name, info in discovered.items():
        symbol_type = info.get('type', 'unknown')
        if symbol_type not in by_type:
            by_type[symbol_type] = []
        by_type[symbol_type].append(name)
    
    for symbol_type, names in by_type.items():
        print(f"{indent_str}{symbol_type.upper()}S ({len(names)}):")
        for name in sorted(names):
            print(f"{indent_str}  - {name}")
            
            # Show methods for classes
            if symbol_type == 'class' and 'methods' in discovered[name]:
                methods = discovered[name]['methods']
                if methods:
                    print(f"{indent_str}    Methods: {', '.join(methods[:5])}")
                    if len(methods) > 5:
                        print(f"{indent_str}    ... and {len(methods) - 5} more")
            
            # Show sub-symbols for namespaces
            if symbol_type == 'namespace' and 'sub_symbols' in discovered[name]:
                sub_symbols = discovered[name]['sub_symbols']
                if sub_symbols:
                    print(f"{indent_str}    Sub-namespace content:")
                    print_symbol_summary(sub_symbols, indent + 2)
        print()


def warmup_rclcpp_symbols(verbose: bool = False) -> None:
    """
    Warm up all rclcpp symbols for better autocomplete experience in IPython.
    This function will access all available symbols to ensure they are loaded into cppyy.
    
    Args:
        verbose: If True, print progress information
    """
    # First ensure rclcpp is loaded
    bringup_rclcpp()
    
    if verbose:
        print("Warming up rclcpp symbols...")
    
    # Common rclcpp classes to ensure are loaded
    known_classes = [
        'Node', 'NodeOptions', 'Publisher', 'Subscription', 'Timer', 'Client', 'Service',
        'QoS', 'KeepAll', 'KeepLast', 'Clock', 'Time', 'Duration', 'Rate',
        'Logger', 'Parameter', 'ParameterValue', 'CallbackGroup', 'Executor',
        'Context'
    ]
    # 'SingleThreadedExecutor', 'MultiThreadedExecutor' not found
    
    # Force access of known classes
    for class_name in known_classes:
        try:
            _ = getattr(cppyy.gbl.rclcpp, class_name)
            if verbose:
                print(f"  ✓ Loaded {class_name}")
        except:
            if verbose:
                print(f"  ✗ Could not load {class_name}")
    
    # Perform deep symbol discovery
    _ = recursive_symbol_discovery(cppyy.gbl.rclcpp, "rclcpp", max_depth=3)
    
    if verbose:
        print(f"Finished warming up {len(dir(cppyy.gbl.rclcpp))} rclcpp symbols")


# Note: if we do:
# import pydoc
# string_with_help_text = pydoc.render_doc(cppyy.gbl.rclcpp.Node, "rclcpp")
# We can get all the info about any element in rclcpp

if __name__ == "__main__":
    warmup_rclcpp_symbols(verbose=True)

