"""
rclcppyy kits: minimal Python glue over C++ libraries via cppyy.

A "kit" wraps a C++ library so it can be driven from short Python, usable for
rapid prototyping (and later freezable / lowerable to native C++). Kits are
optional and each pulls its own C++ dependency, so nothing here is imported by
``rclcppyy/__init__.py`` -- import the kit you want explicitly, e.g.::

    from rclcppyy.kits import bt_kit

Available kits:
    bt_kit   -- BehaviorTree.CPP v4 (requires ros-jazzy-behaviortree-cpp).
    pcl_kit  -- Point Cloud Library (requires pcl + ros-jazzy-pcl-conversions).
    ompl_kit -- Open Motion Planning Library (requires ros-jazzy-ompl).
    nav2_kit -- Nav2 algorithm cores: Costmap2D + NavFn planner, driven from Python
                with no lifecycle servers / pluginlib / tf (requires the nav2 env:
                ros-jazzy-nav2-costmap-2d, ros-jazzy-nav2-navfn-planner).
"""
