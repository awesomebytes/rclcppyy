"""
rclcppyy kits: minimal Python glue over C++ libraries via cppyy.

A "kit" wraps a C++ library so it can be driven from short Python, usable for
rapid prototyping (and later freezable / lowerable to native C++). Kits are
optional and each pulls its own C++ dependency, so nothing here is imported by
``rclcppyy/__init__.py`` -- import the kit you want explicitly, e.g.::

    from rclcppyy.kits import bt_kit

Available kits:
    bt_kit -- BehaviorTree.CPP v4 (requires ros-jazzy-behaviortree-cpp).
"""
