#!/usr/bin/env python
"""
pcl_kit demo 1 -- the minimal path: a NumPy point cloud goes into a PCL
VoxelGrid and comes back out as NumPy, with the PCL C++ API used verbatim.

No ROS here -- just NumPy in, NumPy out. The only kit calls are the bringup and
the two NumPy bridges; ``VoxelGrid`` / ``setInputCloud`` / ``setLeafSize`` /
``filter`` are PCL's own names, called directly on the returned namespace.

Run: pixi run -e pcl demo-pcl-voxel
"""
import numpy as np

from rclcppyy.kits import pcl_kit


def main():
    pcl = pcl_kit.bringup_pcl(with_ros=False)   # NumPy-only, so skip the ROS JIT

    # 100k random points in a 1 m cube.
    points = np.random.default_rng(0).random((100_000, 3), dtype=np.float32)
    cloud = pcl_kit.cloud_from_numpy(points)     # one C++ memcpy into the cloud

    vox = pcl.VoxelGrid[pcl.PointXYZ]()          # PCL's own API, verbatim
    vox.setInputCloud(cloud.makeShared())
    vox.setLeafSize(0.05, 0.05, 0.05)
    downsampled = pcl.PointCloud[pcl.PointXYZ]()
    vox.filter(downsampled)

    out = pcl_kit.cloud_to_numpy(downsampled)    # strided copy back to (M,3)
    print(f"input:  {points.shape[0]:>7} points  (NumPy {points.shape} {points.dtype})")
    print(f"output: {out.shape[0]:>7} points  (NumPy {out.shape} {out.dtype})  after 0.05 m VoxelGrid")


if __name__ == "__main__":
    main()
