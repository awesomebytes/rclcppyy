# rclcppyy

ROS 2 package providing rclcpp bindings via cppyy and examples on how to use cppyy in ROS2.

* Tired of writing python wrappers for your C++ code?
* Missing features from C++ APIs that you'd like to call in Python?
* Do you like to prototype and test in Python but you use a lot of C++ code?

`cppyy` can help you! Cppyy is a Python-C++ bindings library that provides automatic, runtime-based access to C++ code from Python using reflection and just-in-time compilation. It enables seamless interoperability between the two languages, allowing Python to call C++ functions and manipulate C++ objects directly.

This repository aims to expose useful ROS2 C++ (and related) APIs via automatic wrapping with `cppyy`.

## Examples

* Benchmarks (ran on a Intel® Core™ Ultra 7 165H × 22 on "Performance" mode on Ubuntu 24.04)
    * Running a publisher and a subscriber at 1khz
    [!](media/benchmark_pub_sub_1k_hz.png)
        * rclpy uses 15~% CPU for the publisher, and 18~% CPU for the subscriber
        * rclccpyy uses 4~% CPU for the publisher, and 4~% CPU for the subscriber

    * Running a publisher and a subscriber at 10khz
    [!](media/benchmark_pub_sub_1k_hz.png)
        * rclpy uses 86~% CPU for the publisher, and 88~% CPU for the subscriber
        * rclccpyy uses 26~% CPU for the publisher, and 22~% CPU for the subscriber

* The `publisher_member_function.py` [Writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) tutorial using `rclcppyy` as backend. [scripts/ros_tutorials/publisher_member_function.py](scripts/ros_tutorials/publisher_member_function.py).



## Run demos
Easiest way to test by yourself is using a Pixi workspace. TODO: make a repo with the Pixi workspace ready to use, with this repo as a git submodule.

### Build

```bash
cd /path/to/workspace
colcon build --packages-select rclcppyy
source install/setup.bash
```

### Run

```bash
# One on each shell probably
ros2 run rclcppyy bench_pub_rclpy.py 10000
ros2 run rclcppyy bench_sub_rclpy.py

ros2 run rclcppyy bench_pub_rclcppyy.py 10000
ros2 run rclcppyy bench_sub_rclcppyy.py

# Or the tutorial example
ros2 run rclcppyy publisher_member_function.py
```

## Roadmap

[x] Benchmark pub/sub

[x] (WIP) Get rclpy tutorials code to run with rclcppyy backend (Got the publisher one!).

[ ] (WIP) Monkeypatch/substitute rclpy with rclcppyy and make your Python nodes use less CPU!

[ ] (WIP) Generate stubs to get IDE autocompletion.

[ ] Demo images.

[ ] Demo pointclouds.

[ ] Demo Nav2.

[ ] Demo Moveit2.

[ ] Demo ROS control.

[ ] Separate into different packages the base `rclcppyy` and other demos/reusable pieces.


## TODO

* Bring down the bringup of rclcppyy time (currently 2.5s~) by figuring out how to build a `.pcm` + `.so` dictionary that is pre-compiled (or at least compiled just once per machine)