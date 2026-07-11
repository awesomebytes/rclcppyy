#!/usr/bin/env python
"""
Benchmark moveit_kit (full MoveIt C++ via cppyy) against moveit_py (the official
pybind subset) on two hot operations, each library in a fresh subprocess:

  * collision checks / sec -- self-collision of the Panda at random states. Both
    libraries can do this standalone, so it is a fair head-to-head.
  * IK solves / sec -- KDL IK to a fixed reachable pose from random seeds. moveit_kit
    loads the KDL plugin in-process (load_kinematics_solver); moveit_py's standalone
    RobotModel(urdf, srdf) has NO way to instantiate a kinematics solver ("No
    kinematics solver instantiated for group") -- it needs the full MoveItPy/launch
    param plumbing -- so this row is moveit_kit only, which is itself the finding.

Numbers include the Python-loop overhead (setToRandomPositions + update + the call)
and are directional, not a microbenchmark of the C++ call alone. Shared machine:
treat as provisional.

Run: pixi run -e moveit bench-moveit
"""
import argparse
import json
import os
import subprocess
import sys
import time

os.environ.setdefault("ROS_DOMAIN_ID", "48")
HERE = os.path.dirname(os.path.abspath(__file__))


# --------------------------------------------------------------------------
def _panda_paths():
    from ament_index_python.packages import get_package_share_directory
    desc = get_package_share_directory("moveit_resources_panda_description")
    cfg = get_package_share_directory("moveit_resources_panda_moveit_config")
    return (os.path.join(desc, "urdf", "panda.urdf"),
            os.path.join(cfg, "config", "panda.srdf"))


def worker_ours(collision_n, ik_n):
    import cppyy
    from rclcppyy.bringup_rclcpp import bringup_rclcpp
    from rclcppyy.kits import moveit_kit
    moveit = moveit_kit.bringup_moveit(with_kinematics=True)
    rclcpp = bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()
    cfg = moveit_kit.panda_config()
    model = moveit_kit.build_robot_model(cfg.urdf, cfg.srdf)
    jmg = model.getJointModelGroup("panda_arm")
    scene = moveit_kit.planning_scene(model)
    cd = cppyy.gbl.collision_detection
    state = scene.getCurrentStateNonConst()

    req = cd.CollisionRequest()
    state.setToDefaultValues(jmg, "ready")
    state.update()
    scene.checkSelfCollision(req, cd.CollisionResult())            # warm
    t = time.perf_counter()
    for _ in range(collision_n):
        state.setToRandomPositions(jmg)
        state.update()
        scene.checkSelfCollision(req, cd.CollisionResult())
    coll_dt = time.perf_counter() - t

    node = moveit_kit.make_node("bench_ours_ik")
    moveit_kit.load_kinematics_solver(node, model, "panda_arm")
    ik_state = moveit.core.RobotState(model)
    ik_state.setToDefaultValues(jmg, "ready")
    ik_state.update()
    target = cppyy.gbl.Eigen.Isometry3d(ik_state.getGlobalLinkTransform("panda_link8"))
    ik_state.setFromIK(jmg, target, 0.1)                           # warm
    ok = 0
    t = time.perf_counter()
    for _ in range(ik_n):
        ik_state.setToRandomPositions(jmg)
        ik_state.update()
        if ik_state.setFromIK(jmg, target, 0.05):
            ok += 1
    ik_dt = time.perf_counter() - t

    print("RESULT " + json.dumps({
        "lib": "moveit_kit", "collision_n": collision_n, "collision_s": coll_dt,
        "collision_per_s": collision_n / coll_dt,
        "ik_n": ik_n, "ik_s": ik_dt, "ik_per_s": ik_n / ik_dt, "ik_ok": ok,
    }))


def worker_mpy(collision_n, ik_n):
    from moveit.core import robot_model, robot_state, planning_scene, collision_detection
    urdf, srdf = _panda_paths()
    model = robot_model.RobotModel(urdf_xml_path=urdf, srdf_xml_path=srdf)
    scene = planning_scene.PlanningScene(model)
    state = scene.current_state
    req = collision_detection.CollisionRequest()

    state.set_to_default_values("panda_arm", "ready")
    state.update()
    scene.check_self_collision(req, collision_detection.CollisionResult())  # warm
    t = time.perf_counter()
    for _ in range(collision_n):
        state.set_to_random_positions()
        state.update()
        scene.check_self_collision(req, collision_detection.CollisionResult())
    coll_dt = time.perf_counter() - t

    # moveit_py cannot instantiate a kinematics solver from a standalone
    # RobotModel(urdf, srdf); set_from_ik returns False. Record the capability gap.
    rs = robot_state.RobotState(model)
    rs.set_to_default_values("panda_arm", "ready")
    rs.update()
    pose = rs.get_pose("panda_link8")
    rs.set_to_random_positions()
    ik_ok = bool(rs.set_from_ik("panda_arm", pose, "panda_link8", 0.1))

    print("RESULT " + json.dumps({
        "lib": "moveit_py", "collision_n": collision_n, "collision_s": coll_dt,
        "collision_per_s": collision_n / coll_dt,
        "ik_n": 0, "ik_s": 0.0, "ik_per_s": None, "ik_ok": ik_ok,
        "ik_note": "no standalone kinematics solver",
    }))


# --------------------------------------------------------------------------
def run(args):
    print("moveit_kit vs moveit_py: collision checks/sec + IK solves/sec.",
          file=sys.stderr, flush=True)
    print("(Shared machine: provisional, directional not exact.)",
          file=sys.stderr, flush=True)
    results = {}
    for lib in ("ours", "mpy"):
        argv = [sys.executable, "-u", os.path.abspath(__file__), "--worker", lib,
                "--collision-n", str(args.collision_n), "--ik-n", str(args.ik_n)]
        print("  [%s] running ..." % lib, file=sys.stderr, flush=True)
        proc = subprocess.run(argv, capture_output=True, text=True, timeout=args.timeout)
        line = next((ln for ln in proc.stdout.splitlines() if ln.startswith("RESULT ")), None)
        if line is None:
            print("  FAILED (%s):\n%s\n%s" % (lib, proc.stdout[-1500:], proc.stderr[-1500:]),
                  file=sys.stderr, flush=True)
            continue
        results[lib] = json.loads(line[len("RESULT "):])
    _table(results)
    return 0 if len(results) == 2 else 1


def _table(results):
    a = results.get("ours")
    b = results.get("mpy")
    print()
    print("  MoveIt from Python: moveit_kit (full C++ via cppyy) vs moveit_py (subset)")
    print("  " + "=" * 66)
    print("  %-26s %18s %18s" % ("operation", "moveit_kit", "moveit_py"))
    print("  " + "-" * 66)
    if a and b:
        print("  %-26s %15.0f/s %15.0f/s"
              % ("self-collision checks", a["collision_per_s"], b["collision_per_s"]))
        ik = "%.0f/s (%d ok)" % (a["ik_per_s"], a["ik_ok"])
        print("  %-26s %18s %18s" % ("KDL IK solves", ik, "unsupported*"))
    print("  " + "-" * 66)
    print("  * moveit_py's standalone RobotModel(urdf, srdf) instantiates no")
    print("    kinematics solver (set_from_ik -> False); IK needs the full MoveItPy")
    print("    node/param plumbing. moveit_kit loads the KDL plugin in-process.")
    print("  Numbers include Python-loop overhead; shared machine -> provisional.")
    sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--worker", choices=["ours", "mpy"], default=None)
    ap.add_argument("--collision-n", type=int, default=20000)
    ap.add_argument("--ik-n", type=int, default=2000)
    ap.add_argument("--timeout", type=float, default=300.0)
    args = ap.parse_args()
    if args.worker == "ours":
        worker_ours(args.collision_n, args.ik_n)
        return 0
    if args.worker == "mpy":
        worker_mpy(args.collision_n, args.ik_n)
        return 0
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
