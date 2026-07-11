#!/usr/bin/env python
"""
nav2_kit demo 2 -- THE SHOWCASE: your own miniature Nav stack, in one Python file.

A complete navigation loop built from Nav2's algorithm cores plus rclcppyy, with no
lifecycle servers, no pluginlib, no tf -- Python owns the loop, C++ owns the math:

  synthetic world (NumPy)  ->  nav2_costmap_2d::Costmap2D   (real Nav2 grid)
                           ->  nav2_navfn_planner::NavFn     (real Nav2 A* planner)
                           ->  pure-pursuit follow loop      (HONEST split: ~30 lines
                                                              of Python -- Nav2's RPP
                                                              controller is lifecycle-
                                                              coupled, see the REPORT)
                           ->  simulated diff-drive kinematics
                           ->  publish OccupancyGrid + Path + TwistStamped via rclcppyy

So an rviz2 with Fixed Frame "map" shows the map, the plan, and the commanded
velocity live. The messages are real C++ messages published through rclcppyy. The
loop is deadline-bounded and self-contained (no external data).

Planner = C++ (NavFn). Controller = Python (pure pursuit) -- stated plainly because
Nav2's RegulatedPurePursuitController plugin needs a LifecycleNode to configure; only
its header-only regulation math is separable (REPORT §Smac/RPP).

Run: pixi run -e nav2 demo-nav2-stack
"""
import argparse
import math
import os
import time

import numpy as np

os.environ.setdefault("ROS_DOMAIN_ID", "47")

import cppyy                                            # noqa: E402
from rclcppyy.bringup_rclcpp import bringup_rclcpp      # noqa: E402
from rclcppyy.kits import nav2_kit                      # noqa: E402

W = H = 120
RES = 0.05                    # meters/cell -> a 6 m x 6 m world
FRAME = "map"
TOPIC_MAP = "nav2_kit/map"
TOPIC_PATH = "nav2_kit/plan"
TOPIC_CMD = "nav2_kit/cmd_vel"


def make_world():
    """Two rooms separated by a vertical wall with a doorway, plus a box obstacle."""
    g = np.zeros((H, W), dtype=np.uint8)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = nav2_kit.LETHAL_OBSTACLE   # border
    g[:, W // 2] = nav2_kit.LETHAL_OBSTACLE                              # divider
    g[H // 2 - 8:H // 2 + 8, W // 2] = nav2_kit.FREE_SPACE               # doorway
    g[25:40, 75:90] = nav2_kit.LETHAL_OBSTACLE                           # a box
    return g


def cell_to_world(cx, cy):
    return (cx + 0.5) * RES, (cy + 0.5) * RES


def world_to_cell(wx, wy):
    return int(wx / RES), int(wy / RES)


# --- pure-pursuit controller (the honest ~30-line Python half) ----------------
def pure_pursuit(pose, path_xy, idx, lookahead, max_v, max_w):
    """One control step. pose=(x,y,theta); path_xy=(N,2) world points; idx=current
    progress index. Returns (v, w, new_idx). Classic pure pursuit: aim at a lookahead
    point, steer by curvature kappa = 2 sin(alpha) / L."""
    x, y, th = pose
    # advance the progress index to the closest point at/ahead of the robot
    while idx < len(path_xy) - 1 and math.hypot(path_xy[idx][0] - x,
                                                path_xy[idx][1] - y) < lookahead:
        idx += 1
    tx, ty = path_xy[idx]
    dist = math.hypot(tx - x, ty - y)
    alpha = math.atan2(ty - y, tx - x) - th
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))    # wrap to [-pi, pi]
    kappa = 2.0 * math.sin(alpha) / max(dist, 1e-3)
    v = max_v * max(0.15, math.cos(alpha))                  # slow when badly aimed
    w = max(-max_w, min(max_w, v * kappa))
    return v, w, idx


# --- C++ message builders (published through rclcppyy) ------------------------
_GLUE_DONE = False


def _ensure_demo_glue():
    """Compile the OccupancyGrid.data fill helper once. Deferred until after
    bringup_nav2(), because nav_msgs::msg::OccupancyGrid is only declared once
    costmap_2d.hpp (which includes it) has been JIT-parsed."""
    global _GLUE_DONE
    if _GLUE_DONE:
        return
    cppyy.cppdef(r"""
    #include <cstring>
    #include <cstdint>
    namespace rclcppyy_nav2demo {
    inline void fill_occ(nav_msgs::msg::OccupancyGrid& m, uintptr_t src, std::size_t n) {
      m.data.resize(n);
      std::memcpy(m.data.data(), reinterpret_cast<const void*>(src), n);  // int8->int8
    }
    }
    """)
    _GLUE_DONE = True


def _stamp(header):
    """Fill a C++ std_msgs/Header stamp from wall time (rclcpp::Clock::now() comes
    back as a cppyy rclcpp::Time with no .to_msg(); setting sec/nanosec is simplest)."""
    t = time.time()
    header.stamp.sec = int(t)
    header.stamp.nanosec = int((t - int(t)) * 1e9)
    return header


def build_occupancy_grid(grid):
    """A costmap grid -> nav_msgs/OccupancyGrid (int8: 0 free, 100 lethal, -1 unknown)."""
    occ = np.zeros_like(grid, dtype=np.int8)
    occ[grid == nav2_kit.LETHAL_OBSTACLE] = 100
    occ[grid == nav2_kit.NO_INFORMATION] = -1
    occ = np.ascontiguousarray(occ)
    _ensure_demo_glue()
    msg = cppyy.gbl.nav_msgs.msg.OccupancyGrid()
    msg.header.frame_id = FRAME
    _stamp(msg.header)
    msg.info.resolution = RES
    msg.info.width = W
    msg.info.height = H
    msg.info.origin.orientation.w = 1.0
    cppyy.gbl.rclcppyy_nav2demo.fill_occ(msg, occ.ctypes.data, occ.size)
    return msg


def build_path_msg(path_xy):
    msg = cppyy.gbl.nav_msgs.msg.Path()
    msg.header.frame_id = FRAME
    _stamp(msg.header)
    for wx, wy in path_xy:
        pose = cppyy.gbl.geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = FRAME
        pose.pose.position.x = float(wx)
        pose.pose.position.y = float(wy)
        pose.pose.orientation.w = 1.0
        msg.poses.push_back(pose)
    return msg


def build_twist(v, w):
    msg = cppyy.gbl.geometry_msgs.msg.TwistStamped()
    msg.header.frame_id = "base_link"
    _stamp(msg.header)
    msg.twist.linear.x = float(v)
    msg.twist.angular.z = float(w)
    return msg


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--rate", type=float, default=20.0, help="control rate (Hz)")
    ap.add_argument("--duration", type=float, default=20.0, help="max run window (s)")
    args = ap.parse_args()

    # ---- plan with Nav2's cores -------------------------------------------------
    nav2_kit.bringup_nav2()
    nav2_kit.warmup()
    grid = make_world()
    costmap = nav2_kit.costmap_from_numpy(grid, resolution=RES)
    start_cell, goal_cell = (18, H // 2), (100, 35)
    path_cells = nav2_kit.plan_navfn(costmap, start_cell, goal_cell)
    if path_cells is None:
        print("No plan found; aborting.")
        return
    path_xy = [cell_to_world(cx, cy) for cx, cy in path_cells]
    print(f"Planned {len(path_xy)} waypoints "
          f"{cell_to_world(*start_cell)} -> {cell_to_world(*goal_cell)} "
          f"(NavFn, C++).", flush=True)

    # ---- ROS 2 bringup (rclcppyy) ----------------------------------------------
    rclcpp = bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()
    cppyy.include("nav_msgs/msg/path.hpp")
    cppyy.include("geometry_msgs/msg/pose_stamped.hpp")
    cppyy.include("geometry_msgs/msg/twist_stamped.hpp")
    from nav_msgs.msg import OccupancyGrid, Path        # register topic types
    from geometry_msgs.msg import TwistStamped

    node = rclcpp.Node("nav2_kit_own_stack")
    map_pub = node.create_publisher(OccupancyGrid, TOPIC_MAP, 1)
    path_pub = node.create_publisher(Path, TOPIC_PATH, 1)
    cmd_pub = node.create_publisher(TwistStamped, TOPIC_CMD, 10)

    map_pub.publish(build_occupancy_grid(grid))
    path_pub.publish(build_path_msg(path_xy))

    # ---- follow loop: pure pursuit + diff-drive sim ----------------------------
    gx, gy = cell_to_world(*goal_cell)
    sx, sy = path_xy[0]
    theta0 = math.atan2(path_xy[1][1] - sy, path_xy[1][0] - sx)
    pose = [sx, sy, theta0]
    idx = 0
    dt = 1.0 / args.rate
    lookahead, max_v, max_w = 0.30, 0.6, 2.0
    reached = False
    steps = int(args.duration * args.rate)

    for step in range(steps):
        dist_goal = math.hypot(gx - pose[0], gy - pose[1])
        if dist_goal < 0.10:
            reached = True
            break
        v, w, idx = pure_pursuit(pose, path_xy, idx, lookahead, max_v, max_w)
        # integrate simulated unicycle/diff-drive kinematics
        pose[0] += v * math.cos(pose[2]) * dt
        pose[1] += v * math.sin(pose[2]) * dt
        pose[2] += w * dt

        cmd_pub.publish(build_twist(v, w))
        if step % 10 == 0:                          # re-publish map/plan for late rviz
            map_pub.publish(build_occupancy_grid(grid))
            path_pub.publish(build_path_msg(path_xy))
            cell = world_to_cell(pose[0], pose[1])
            print(f"STEP {step:4d} pose=({pose[0]:.2f},{pose[1]:.2f},"
                  f"{math.degrees(pose[2]):6.1f} deg) cell={cell} "
                  f"cmd=(v={v:.2f},w={w:+.2f}) dist_to_goal={dist_goal:.2f}",
                  flush=True)
        time.sleep(dt)

    # stop the robot
    cmd_pub.publish(build_twist(0.0, 0.0))
    if reached:
        print(f"GOAL REACHED at ({pose[0]:.2f},{pose[1]:.2f}) "
              f"in {step} steps (~{step * dt:.1f}s sim).", flush=True)
    else:
        print(f"Deadline hit; nearest approach dist_to_goal="
              f"{math.hypot(gx - pose[0], gy - pose[1]):.2f}.", flush=True)
    print("SUMMARY done", flush=True)


if __name__ == "__main__":
    main()
