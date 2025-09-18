#! /usr/bin/env python3
import os
import rclcppyy; rclcppyy.enable_cpp_acceleration()
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import cppyy
from sensor_msgs.msg import PointCloud2

def prepare():
    print("including pcl headers")
    cppyy.add_include_path(f"{os.environ["CONDA_PREFIX"]}/include/pcl-1.15")
    cppyy.add_include_path(f"{os.environ["CONDA_PREFIX"]}/include/eigen3")
    cppyy.include("pcl/point_types.h")
    cppyy.include("pcl/point_cloud.h")
    cppyy.include("pcl/filters/voxel_grid.h")
    cppyy.include("pcl/filters/impl/voxel_grid.hpp")
    cppyy.include("pcl_conversions/pcl_conversions.h")
    print("included pcl headers")

    # cppyy: VoxelGrid wrapper for ROS2 PointCloud2 in/out via pcl_conversions
    print("defining voxelgrid_filter_pointcloud2")
    cppyy.cppdef("""
    #include <pcl/point_types.h>
    #include <pcl/point_cloud.h>
    #include <pcl/filters/voxel_grid.h>
    #include <pcl/filters/impl/voxel_grid.hpp>
    #include <pcl_conversions/pcl_conversions.h>
    #include <sensor_msgs/msg/point_cloud2.hpp>

    extern "C" void voxelgrid_filter_pointcloud2(const sensor_msgs::msg::PointCloud2& in_msg,
                                                float leaf_x, float leaf_y, float leaf_z,
                                                sensor_msgs::msg::PointCloud2& out_msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(in_msg, *cloud);

        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(leaf_x, leaf_y, leaf_z);

        pcl::PointCloud<pcl::PointXYZ> out_cloud;
        voxel.filter(out_cloud);

        pcl::toROSMsg(out_cloud, out_msg);
        out_msg.header = in_msg.header; // preserve frame/time
    }
    """)
    print("defined voxelgrid_filter_pointcloud2")

def voxelgrid_downsample_msg(msg: PointCloud2, leaf=0.1, leaf_y=None, leaf_z=None) -> PointCloud2:
    leaf_y = leaf if leaf_y is None else leaf_y
    leaf_z = leaf if leaf_z is None else leaf_z
    out = PointCloud2()
    cppyy.gbl.voxelgrid_filter_pointcloud2(msg, float(leaf), float(leaf_y), float(leaf_z), out)
    return out

class ImageSubscriber(Node):
    def __init__(self, topic_in: str, topic_out: str, leaf: float = 0.25) -> None:
        super().__init__("pointcloud_voxelgrid")
        self.leaf = leaf
        self.publisher = self.create_publisher(PointCloud2, topic_out, qos_profile_sensor_data)
        # Subscribe to the same topic as the demo publisher
        self.subscription = self.create_subscription(
            PointCloud2,
            topic_in,
            self._on_pointcloud,
            10,
        )
        print("subscription created")

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        msg = voxelgrid_downsample_msg(msg, leaf=self.leaf)
        self.publisher.publish(msg)

def run_voxelgrid(topic_in: str, topic_out: str, leaf: float = 0.25) -> None:
    """Run the voxelgrid downsampling node."""
    prepare()
    rclpy.init()
    node = ImageSubscriber(topic_in, topic_out, leaf)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    run_voxelgrid("/lexus3/os_center/points", "/voxelgrid", 0.5)


