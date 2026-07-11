"""
moveit_kit -- drive the full MoveIt C++ (moveit_cpp) API from Python via cppyy.

MoveIt *has* an official Python binding, ``moveit_py`` -- and it is an explicit,
hand-maintained **subset**: it wraps ``MoveItPy`` / ``PlanningComponent`` and a
curated slice of ``RobotState`` / ``PlanningScene``, but leaves large parts of the
C++ API unreachable (goal-tolerance setters, full ACM manipulation, constraint
surfaces beyond the ``construct*`` helpers, direct plugin construction, ...). This
kit skips the curation: it is a thin cppyy glue layer that **mirrors MoveIt's own
C++ API** -- you build ``moveit::core::RobotModel``, ``planning_scene::PlanningScene``,
call ``RobotState::setFromIK`` / ``checkSelfCollision`` / a real OMPL
``PlannerManager`` -- the same names and shapes as the MoveIt C++ tutorials,
against the MoveIt that is already installed. Nothing is generated.

The novel pattern (see docs/moveit_kit/REPORT.md): MoveIt's C++ stack is
**parameter- and plugin-driven**. Kinematics (the KDL plugin) and planning (the
OMPL plugin) load via ``pluginlib`` and read their configuration from ROS node
parameters. This kit boots that from a Python-created ``rclcpp::Node`` whose
parameter overrides are assembled from the panda config YAMLs, and loads the
plugins in-process via ``pluginlib::ClassLoader``. Two frictions shape the design:

  * MoveIt's ``generate_parameter_library``-generated headers (``*_parameters.hpp``,
    pulled by ``robot_model_loader.hpp`` / ``planning_pipeline.hpp`` /
    ``moveit_cpp.hpp``) use modern C++ + ``fmt`` + ``rsl`` that **crash Cling's
    parser**. So the kit does NOT include those convenience headers; it loads the
    plugins itself against the *clean* base headers (``kinematics_base.hpp``,
    ``planning_interface.hpp``), which is the same pluginlib mechanic MoveItCpp
    uses internally, minus the header that will not parse.
  * A pluginlib-loaded plugin owns process-global state (the ``ClassLoader`` and the
    plugin instance) whose C++ destructor, if it runs during/after cppyy's Cling
    teardown, **segfaults**. The kit registers a teardown (via
    ``cppyy_kit.register_teardown``) that resets the plugin instances *before* Cling
    goes away -- deterministic clean exit. (A benign ``class_loader`` "SEVERE
    WARNING ... will NOT be unloaded" may still print at exit; it is class_loader
    choosing the safe action, not an error.)

Staged bringup (each stage is idempotent; later stages imply earlier ones):

    from rclcppyy.kits import moveit_kit
    moveit = moveit_kit.bringup_moveit()                 # PARSE layer: model+state+scene
    cfg = moveit_kit.panda_config()
    model = moveit_kit.build_robot_model(cfg.urdf, cfg.srdf)
    state = moveit.core.RobotState(model)
    state.setToDefaultValues(model.getJointModelGroup("panda_arm"), "ready")
    state.update()
    print(state.getGlobalLinkTransform("panda_link8").translation()[0])   # FK

    moveit_kit.bringup_moveit(with_kinematics=True)      # + pluginlib KDL IK
    node = moveit_kit.make_node("ik")
    moveit_kit.load_kinematics_solver(node, model, "panda_arm")
    state.setFromIK(model.getJointModelGroup("panda_arm"), target_isometry, 0.1)

    moveit_kit.bringup_moveit(with_planning=True)        # + real OMPL PlannerManager
    node = moveit_kit.make_node("plan", moveit_kit.parameter_overrides(cfg.ompl, "ompl"))
    planner = moveit_kit.load_planner(node, model)
    result = moveit_kit.plan_joint_goal(planner, scene, model, "panda_arm", goal_state)

See docs/moveit_kit/MOVEIT_KIT.md for the API cheat sheet and WHY.md for the
moveit_py contrast.
"""
import os

import cppyy

from rclcppyy.kits import cppyy_kit

# --- PARSE layer: build a RobotModel from URDF+SRDF strings, FK/collision. ---
# These headers all parse cleanly in Cling (no generate_parameter_library).
_PARSE_HEADERS = (
    "urdf_parser/urdf_parser.h",
    "srdfdom/model.h",
    "moveit/robot_model/robot_model.hpp",
    "moveit/robot_state/robot_state.hpp",
    "moveit/robot_state/conversions.hpp",
    "moveit/planning_scene/planning_scene.hpp",
    "moveit/collision_detection/collision_common.hpp",
    "geometric_shapes/shapes.h",
)
_PARSE_LIBS = (
    "liburdfdom_model.so", "libsrdfdom.so",
    "libmoveit_robot_model.so", "libmoveit_robot_state.so",
    "libmoveit_collision_detection.so", "libmoveit_collision_detection_fcl.so",
    "libmoveit_planning_scene.so", "libmoveit_utils.so",
)

# --- KINEMATICS layer: load the KDL plugin via pluginlib for IK. ---
_KINEMATICS_HEADERS = (
    "moveit/kinematics_base/kinematics_base.hpp",
    "pluginlib/class_loader.hpp",
)
_KINEMATICS_LIBS = ("libmoveit_kinematics_base.so", "libclass_loader.so")

# --- PLANNING layer: load the OMPL PlannerManager plugin via pluginlib. ---
_PLANNING_HEADERS = (
    "moveit/planning_interface/planning_interface.hpp",
    "moveit/kinematic_constraints/utils.hpp",
    "moveit/robot_trajectory/robot_trajectory.hpp",
    "moveit_msgs/msg/display_trajectory.hpp",
    "pluginlib/class_loader.hpp",
)
_PLANNING_LIBS = (
    "libmoveit_kinematic_constraints.so", "libmoveit_planning_interface.so",
    "libmoveit_robot_trajectory.so", "libompl.so", "libmoveit_ompl_interface.so",
)

KDL_PLUGIN = "kdl_kinematics_plugin/KDLKinematicsPlugin"
OMPL_PLUGIN = "ompl_interface/OMPLPlanner"

# C++ glue for the PARSE layer: an Eigen pose helper (Eigen block assignment --
# ``pose.translation()[i] = v`` -- is not supported across cppyy, so build the
# isometry in C++).
_PARSE_GLUE = r"""
namespace moveit_kit_cpp {
  Eigen::Isometry3d make_pose(double x, double y, double z,
                              double qx, double qy, double qz, double qw) {
    Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
    p.translation() = Eigen::Vector3d(x, y, z);
    Eigen::Quaterniond q(qw, qx, qy, qz);
    q.normalize();
    p.linear() = q.toRotationMatrix();
    return p;
  }
}
"""

# C++ glue for the KINEMATICS layer: pluginlib loader + ordered teardown. The
# ClassLoader and plugin instance are process-global statics; teardown resets them
# before Cling teardown (else the plugin dtor segfaults at exit).
_KINEMATICS_GLUE = r"""
#include <memory>
#include <vector>
#include <string>
namespace moveit_kit_cpp {
  static std::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> g_kin_loader;
  static kinematics::KinematicsBasePtr g_kin_solver;

  bool load_kinematics(const rclcpp::Node::SharedPtr& node,
                       const moveit::core::RobotModelPtr& model,
                       const std::string& plugin, const std::string& group,
                       const std::string& base, const std::string& tip, double res) {
    if (!g_kin_loader)
      g_kin_loader = std::make_shared<pluginlib::ClassLoader<kinematics::KinematicsBase>>(
          "moveit_core", "kinematics::KinematicsBase");
    std::shared_ptr<kinematics::KinematicsBase> solver(
        g_kin_loader->createUniqueInstance(plugin).release());
    std::vector<std::string> tips{tip};
    if (!solver->initialize(node, *model, group, base, tips, res))
      return false;
    g_kin_solver = solver;
    moveit::core::SolverAllocatorFn fn =
        [solver](const moveit::core::JointModelGroup*) { return solver; };
    model->getJointModelGroup(group)->setSolverAllocators(fn);
    return true;
  }

  void teardown_kinematics() { g_kin_solver.reset(); g_kin_loader.reset(); }
}
"""

# C++ glue for the PLANNING layer: OMPL PlannerManager loader + a plan helper that
# builds the request, runs the context, and returns the trajectory + status.
_PLANNING_GLUE = r"""
#include <memory>
namespace moveit_kit_cpp {
  static std::shared_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> g_plan_loader;
  static planning_interface::PlannerManagerPtr g_planner;

  struct PlanResult {
    bool ok = false;
    int error_code = 0;
    double planning_time = 0.0;
    robot_trajectory::RobotTrajectoryPtr trajectory;
  };

  planning_interface::PlannerManagerPtr load_planner(
      const rclcpp::Node::SharedPtr& node,
      const moveit::core::RobotModelPtr& model,
      const std::string& plugin, const std::string& ns) {
    if (!g_plan_loader)
      g_plan_loader = std::make_shared<pluginlib::ClassLoader<planning_interface::PlannerManager>>(
          "moveit_core", "planning_interface::PlannerManager");
    planning_interface::PlannerManagerPtr pm(
        g_plan_loader->createUniqueInstance(plugin).release());
    if (!pm->initialize(model, node, ns))
      return nullptr;
    g_planner = pm;
    return pm;
  }

  // Run a MotionPlanRequest whose goal_constraints are already set, filling result.
  PlanResult run_request(const planning_interface::PlannerManagerPtr& pm,
                         const planning_scene::PlanningScenePtr& scene,
                         planning_interface::MotionPlanRequest& req) {
    PlanResult out;
    // start state = the scene's current state (avoids the "empty JointState" warn)
    moveit::core::robotStateToRobotStateMsg(scene->getCurrentState(), req.start_state);
    moveit_msgs::msg::MoveItErrorCodes ec;
    auto ctx = pm->getPlanningContext(scene, req, ec);
    if (!ctx) { out.error_code = ec.val; return out; }
    planning_interface::MotionPlanResponse res;
    ctx->solve(res);
    out.error_code = res.error_code.val;
    out.planning_time = res.planning_time;
    out.trajectory = res.trajectory;
    out.ok = (res.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
             && res.trajectory && res.trajectory->getWayPointCount() > 0;
    return out;
  }

  PlanResult plan_joint_goal(const planning_interface::PlannerManagerPtr& pm,
                             const planning_scene::PlanningScenePtr& scene,
                             const moveit::core::RobotModelPtr& model,
                             const std::string& group,
                             const moveit::core::RobotState& goal_state,
                             const std::string& planner_id, double timeout) {
    planning_interface::MotionPlanRequest req;
    req.group_name = group;
    req.planner_id = planner_id;
    req.allowed_planning_time = timeout;
    const auto* jmg = model->getJointModelGroup(group);
    req.goal_constraints.push_back(
        kinematic_constraints::constructGoalConstraints(goal_state, jmg));
    return run_request(pm, scene, req);
  }

  PlanResult plan_pose_goal(const planning_interface::PlannerManagerPtr& pm,
                            const planning_scene::PlanningScenePtr& scene,
                            const std::string& group, const std::string& link,
                            double x, double y, double z,
                            double qx, double qy, double qz, double qw,
                            double tol_pos, double tol_angle,
                            const std::string& planner_id, double timeout) {
    planning_interface::MotionPlanRequest req;
    req.group_name = group;
    req.planner_id = planner_id;
    req.allowed_planning_time = timeout;
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = scene->getPlanningFrame();
    ps.pose.position.x = x; ps.pose.position.y = y; ps.pose.position.z = z;
    ps.pose.orientation.x = qx; ps.pose.orientation.y = qy;
    ps.pose.orientation.z = qz; ps.pose.orientation.w = qw;
    req.goal_constraints.push_back(
        kinematic_constraints::constructGoalConstraints(link, ps, tol_pos, tol_angle));
    return run_request(pm, scene, req);
  }

  // Build an rviz-compatible moveit_msgs::DisplayTrajectory from a plan result.
  moveit_msgs::msg::DisplayTrajectory to_display_trajectory(
      const PlanResult& r, const planning_scene::PlanningScenePtr& scene,
      const std::string& model_id) {
    moveit_msgs::msg::DisplayTrajectory dt;
    dt.model_id = model_id;
    moveit::core::robotStateToRobotStateMsg(scene->getCurrentState(), dt.trajectory_start);
    if (r.trajectory) {
      moveit_msgs::msg::RobotTrajectory rt;
      r.trajectory->getRobotTrajectoryMsg(rt);
      dt.trajectory.push_back(rt);
    }
    return dt;
  }

  void teardown_planning() { g_planner.reset(); g_plan_loader.reset(); }
}
"""

_MOVEIT = None
_PARSE_DONE = False
_KINEMATICS_DONE = False
_PLANNING_DONE = False


def _lib_dir():
    return os.path.join(os.environ["CONDA_PREFIX"], "lib")


def _add_moveit_includes():
    """Add MoveIt/ROS include paths (reuses rclcppyy's ament-index helper, which
    registers ``include/<pkg>`` for every package -- the layout MoveIt headers such
    as ``<urdf/model.h>`` and ``<moveit/.../*.hpp>`` expect) plus eigen3."""
    from rclcppyy.bringup_rclcpp import add_ros2_include_paths
    add_ros2_include_paths()
    cppyy.add_include_path(os.path.join(os.environ["CONDA_PREFIX"], "include", "eigen3"))


def _ensure_parse():
    """Bring up the PARSE layer (RobotModel/RobotState/PlanningScene + FCL). No
    node, no plugins. Idempotent."""
    global _MOVEIT, _PARSE_DONE
    if _PARSE_DONE:
        return
    _add_moveit_includes()
    cppyy.add_library_path(_lib_dir())
    for header in _PARSE_HEADERS:
        cppyy.include(header)
    cppyy_kit.load_libraries(_PARSE_LIBS, [_lib_dir()])
    cppyy.cppdef(_PARSE_GLUE)
    _MOVEIT = cppyy.gbl.moveit
    _PARSE_DONE = True


def _ensure_kinematics():
    """Add the KINEMATICS layer: the KDL plugin loaded via pluginlib. Idempotent."""
    global _KINEMATICS_DONE
    if _KINEMATICS_DONE:
        return
    _ensure_parse()
    for header in _KINEMATICS_HEADERS:
        cppyy.include(header)
    cppyy_kit.load_libraries(_KINEMATICS_LIBS, [_lib_dir()])
    cppyy.cppdef(_KINEMATICS_GLUE)
    cppyy_kit.register_teardown(cppyy.gbl.moveit_kit_cpp.teardown_kinematics)
    _KINEMATICS_DONE = True


def _ensure_planning():
    """Add the PLANNING layer: the OMPL PlannerManager loaded via pluginlib.
    Idempotent."""
    global _PLANNING_DONE
    if _PLANNING_DONE:
        return
    _ensure_parse()
    for header in _PLANNING_HEADERS:
        cppyy.include(header)
    cppyy_kit.load_libraries(_PLANNING_LIBS, [_lib_dir()])
    cppyy.cppdef(_PLANNING_GLUE)
    cppyy_kit.register_teardown(cppyy.gbl.moveit_kit_cpp.teardown_planning)
    _PLANNING_DONE = True


def bringup_moveit(with_kinematics=False, with_planning=False):
    """
    Bring up MoveIt under cppyy and return the ``moveit`` namespace. Idempotent.

    Stages (each implies the ones before it):

    * **parse** (always): JIT-includes the RobotModel / RobotState / PlanningScene /
      collision (FCL) headers -- all Cling-parseable -- and loads their ``.so`` set.
      Enough for ``build_robot_model``, FK, and collision checking. No node, no
      plugins.
    * **kinematics** (``with_kinematics=True``): adds the KDL kinematics plugin via
      pluginlib, so ``load_kinematics_solver`` + ``RobotState::setFromIK`` work.
    * **planning** (``with_planning=True``): adds MoveIt's real OMPL
      ``PlannerManager`` plugin via pluginlib, so ``load_planner`` +
      ``plan_joint_goal`` work.

    The heavier stages are gated because they (a) JIT more headers and (b) register
    a plugin teardown; a parse-only user (FK / collision) skips both. Returns
    ``cppyy.gbl.moveit`` -- use ``moveit.core.RobotState`` etc. directly; the sibling
    namespaces ``cppyy.gbl.planning_scene`` / ``cppyy.gbl.collision_detection`` /
    ``cppyy.gbl.kinematic_constraints`` are also live once brought up.
    """
    _ensure_parse()
    if with_kinematics:
        _ensure_kinematics()
    if with_planning:
        _ensure_planning()
    return _MOVEIT


def moveit():
    """The ``cppyy.gbl.moveit`` namespace (``moveit.core.*``), bringing up the parse
    layer if needed."""
    _ensure_parse()
    return _MOVEIT


class PandaConfig:
    """The panda test model's config strings + parsed YAMLs, located via the ament
    index (``moveit_resources_panda_description`` / ``_moveit_config``)."""

    def __init__(self, urdf, srdf, kinematics, ompl, joint_limits):
        self.urdf = urdf                    # URDF XML string
        self.srdf = srdf                    # SRDF XML string
        self.kinematics = kinematics        # parsed kinematics.yaml (dict)
        self.ompl = ompl                    # parsed ompl_planning.yaml (dict)
        self.joint_limits = joint_limits    # parsed joint_limits.yaml (dict)


def panda_config():
    """Locate and read the panda test model's config: URDF + SRDF strings and the
    kinematics / OMPL / joint-limit YAMLs (parsed to dicts). This is real friction,
    not sugar -- MoveIt's plugins are configured entirely from these, and the paths
    live in two different ament packages."""
    import yaml
    from ament_index_python.packages import get_package_share_directory
    desc = get_package_share_directory("moveit_resources_panda_description")
    cfg = get_package_share_directory("moveit_resources_panda_moveit_config")

    def _read(path):
        with open(path) as handle:
            return handle.read()

    def _yaml(path):
        with open(path) as handle:
            return yaml.safe_load(handle)

    return PandaConfig(
        urdf=_read(os.path.join(desc, "urdf", "panda.urdf")),
        srdf=_read(os.path.join(cfg, "config", "panda.srdf")),
        kinematics=_yaml(os.path.join(cfg, "config", "kinematics.yaml")),
        ompl=_yaml(os.path.join(cfg, "config", "ompl_planning.yaml")),
        joint_limits=_yaml(os.path.join(cfg, "config", "joint_limits.yaml")),
    )


def build_robot_model(urdf_string, srdf_string):
    """
    Build a ``moveit::core::RobotModel`` from URDF + SRDF strings -- the parse-only
    path, no node and no parameters.

    Parses the URDF (``urdf::parseURDF``), inits an ``srdf::Model`` from it, and
    constructs the ``RobotModel``. Returns a ``RobotModelPtr`` (shared_ptr); use
    MoveIt's own API on it (``getJointModelGroup``, ``getLinkModelNames``, ...).
    """
    _ensure_parse()
    urdf_model = cppyy.gbl.urdf.parseURDF(urdf_string)
    if not urdf_model:
        raise cppyy_kit.CppyyKitError("moveit_kit.build_robot_model: URDF failed to parse")
    srdf_model = cppyy.gbl.std.make_shared["srdf::Model"]()
    # cppyy dereferences the shared_ptr to bind the const ModelInterface& parameter.
    srdf_model.initString(urdf_model, srdf_string)
    return cppyy.gbl.std.make_shared["moveit::core::RobotModel"](urdf_model, srdf_model)


def planning_scene(robot_model):
    """A ``planning_scene::PlanningScene`` for ``robot_model`` (FCL is the default
    collision detector). Mirror helper -- equivalent to
    ``cppyy.gbl.std.make_shared['planning_scene::PlanningScene'](model)``."""
    _ensure_parse()
    return cppyy.gbl.std.make_shared["planning_scene::PlanningScene"](robot_model)


def parameter_value(value):
    """A Python scalar / homogeneous list -> ``rclcpp::ParameterValue``. Handles
    bool / int / float / str and lists thereof (the value shapes a config YAML
    produces)."""
    _ensure_parse()
    pv = cppyy.gbl.rclcpp.ParameterValue
    if isinstance(value, bool):
        return pv(value)
    if isinstance(value, int):
        return pv(int(value))
    if isinstance(value, float):
        return pv(float(value))
    if isinstance(value, str):
        return pv(cppyy.gbl.std.string(value))
    if isinstance(value, (list, tuple)):
        if all(isinstance(v, str) for v in value):
            return pv(cppyy.gbl.std.vector["std::string"](
                [cppyy.gbl.std.string(v) for v in value]))
        if all(isinstance(v, bool) for v in value):
            return pv(cppyy.gbl.std.vector["bool"](list(value)))
        if all(isinstance(v, int) for v in value):
            return pv(cppyy.gbl.std.vector["int64_t"](list(value)))
        return pv(cppyy.gbl.std.vector["double"]([float(v) for v in value]))
    raise cppyy_kit.CppyyKitError(
        "moveit_kit.parameter_value: unsupported value %r" % (value,))


def parameter_overrides(tree, prefix=""):
    """
    Flatten a nested dict (a parsed config YAML) into a ``std::vector<rclcpp::
    Parameter>`` with dotted names under ``prefix`` -- the ROS / generate_parameter_
    library convention (``ompl_planning.yaml`` becomes ``ompl.panda_arm.
    planner_configs`` etc.).

    This is the param-assembly friction the plugin-driven stack forces: MoveIt's
    kinematics and OMPL plugins read their whole configuration from node parameters,
    so a Python-created node must carry the config the launch files would otherwise
    inject. Pass the result to ``make_node`` (or ``NodeOptions.parameter_overrides``).
    """
    _ensure_parse()
    out = cppyy.gbl.std.vector["rclcpp::Parameter"]()
    param = cppyy.gbl.rclcpp.Parameter

    def _walk(node, pfx):
        for key, value in node.items():
            name = "%s.%s" % (pfx, key) if pfx else str(key)
            if isinstance(value, dict):
                _walk(value, name)
            else:
                out.push_back(param(name, parameter_value(value)))

    _walk(tree, prefix)
    return out


def make_node(name, overrides=None):
    """
    Create an ``rclcpp::Node::SharedPtr`` with parameter overrides -- the node
    MoveIt's plugins read their config from.

    ``overrides`` is a ``std::vector<rclcpp::Parameter>`` (from ``parameter_overrides``)
    or ``None``. ``automatically_declare_parameters_from_overrides`` is enabled so the
    plugins can read the overrides without pre-declaring each one. Requires rclcpp to
    be initialized (``bringup_rclcpp(); rclcpp.init()``)."""
    _ensure_parse()
    opts = cppyy.gbl.rclcpp.NodeOptions()
    if overrides is not None:
        opts.automatically_declare_parameters_from_overrides(True)
        opts.parameter_overrides(overrides)
    return cppyy.gbl.std.make_shared["rclcpp::Node"](name, opts)


def load_kinematics_solver(node, robot_model, group, plugin=KDL_PLUGIN,
                           base_frame=None, tip_frame=None, search_resolution=0.005):
    """
    Load a kinematics plugin (default: the KDL solver) via pluginlib and attach it to
    ``group`` so ``RobotState::setFromIK`` works.

    The plugin is loaded from a C++ ``pluginlib::ClassLoader`` (bypassing MoveIt's
    ``kinematics_plugin_loader.hpp``, which pulls a Cling-crashing generated param
    header), initialized against ``node`` (the plugin declares its own param defaults
    under the group namespace, so a plain node suffices), and wired onto the group's
    solver allocator. ``base_frame`` / ``tip_frame`` default to the group's root and
    the last link of its chain. Returns True on success.
    """
    _ensure_kinematics()
    jmg = robot_model.getJointModelGroup(group)
    if base_frame is None:
        base_frame = str(robot_model.getRootLinkName())
    if tip_frame is None:
        tip_frame = str(list(jmg.getLinkModelNames())[-1])
    with cppyy_kit.first_use("moveit_kit.load_kinematics_solver", "moveit_kit.warmup()"):
        return bool(cppyy.gbl.moveit_kit_cpp.load_kinematics(
            node, robot_model, plugin, group, base_frame, tip_frame,
            float(search_resolution)))


def load_planner(node, robot_model, plugin=OMPL_PLUGIN, parameter_namespace="ompl"):
    """
    Load MoveIt's real motion-planning ``PlannerManager`` plugin (default: OMPL) via
    pluginlib and initialize it against ``node``.

    ``node`` must carry the planner's parameters under ``parameter_namespace`` (build
    them with ``parameter_overrides(cfg.ompl, "ompl")`` and ``make_node``). Bypasses
    ``planning_pipeline.hpp`` (Cling-crashing generated header) and loads the plugin
    directly, as MoveItCpp does internally. Returns the ``PlannerManagerPtr`` (or
    raises if it fails to initialize).
    """
    _ensure_planning()
    with cppyy_kit.first_use("moveit_kit.load_planner", "moveit_kit.warmup()"):
        pm = cppyy.gbl.moveit_kit_cpp.load_planner(
            node, robot_model, plugin, parameter_namespace)
    if not pm:
        raise cppyy_kit.CppyyKitError(
            "moveit_kit.load_planner: PlannerManager '%s' failed to initialize" % plugin)
    return pm


def plan_joint_goal(planner, scene, robot_model, group, goal_state,
                    planner_id="", allowed_planning_time=5.0):
    """
    Plan a collision-aware joint-space motion for ``group`` from ``scene``'s current
    state to ``goal_state``, using ``planner`` (a loaded ``PlannerManager``).

    Builds the ``MotionPlanRequest`` (start = scene current state, goal =
    joint constraints from ``goal_state`` via ``constructGoalConstraints``), gets a
    planning context, and solves. Returns the C++ ``PlanResult`` struct (fields:
    ``ok``, ``error_code``, ``planning_time``, ``trajectory`` -- a
    ``robot_trajectory::RobotTrajectoryPtr``). ``planner_id`` empty lets the config
    pick the group's default (RRTConnect for panda_arm).
    """
    _ensure_planning()
    return cppyy.gbl.moveit_kit_cpp.plan_joint_goal(
        planner, scene, robot_model, group, goal_state, planner_id,
        float(allowed_planning_time))


def plan_pose_goal(planner, scene, group, link, target_pose,
                   tolerance_pos=1e-3, tolerance_angle=1e-2,
                   planner_id="", allowed_planning_time=5.0):
    """
    Plan a collision-aware motion for ``group`` from ``scene``'s current state to a
    Cartesian **pose goal** for ``link`` (the OMPL pipeline samples IK internally).

    ``target_pose`` is ``(x, y, z, qx, qy, qz, qw)``. Builds pose goal constraints via
    ``constructGoalConstraints(link, PoseStamped, tolerance_pos, tolerance_angle)`` --
    note the explicit **goal tolerances** (position metres, orientation radians),
    something moveit_py's high-level ``set_goal`` does not surface. Returns the same
    ``PlanResult`` struct as ``plan_joint_goal``.

    Requires the group's **kinematics solver** to be loaded
    (``load_kinematics_solver``) in addition to the planner: OMPL samples the pose
    goal into joint states via IK, so a pose goal without a solver fails.
    """
    _ensure_planning()
    x, y, z, qx, qy, qz, qw = target_pose
    return cppyy.gbl.moveit_kit_cpp.plan_pose_goal(
        planner, scene, group, link, float(x), float(y), float(z),
        float(qx), float(qy), float(qz), float(qw),
        float(tolerance_pos), float(tolerance_angle), planner_id,
        float(allowed_planning_time))


def display_trajectory(plan_result, scene, model_id="panda"):
    """
    Build an rviz-compatible ``moveit_msgs::msg::DisplayTrajectory`` (C++ message)
    from a ``PlanResult`` and the ``scene`` it was planned in.

    Fills ``model_id``, ``trajectory_start`` (the scene's current state) and one
    ``trajectory`` entry (the plan converted via ``RobotTrajectory::getRobotTrajectoryMsg``).
    Publish it on ``/display_planned_path`` via rclcppyy and RViz's MotionPlanning
    display renders the motion. Returns the C++ message; hand it straight to a
    publisher's ``publish`` (rclcppyy publishes C++ messages with no conversion)."""
    _ensure_planning()
    return cppyy.gbl.moveit_kit_cpp.to_display_trajectory(plan_result, scene, model_id)


# The GroupStateValidityCallbackFn signature RobotState::setFromIK accepts to make
# IK collision-aware. It is a pointer/const-pointer form cppyy_kit.callback's
# type-hint inference cannot produce, so the kit pins the exact string (mirroring
# ompl_kit's validity_checker). moveit_py's set_from_ik has NO such callback slot.
_IK_VALIDITY_SIG = ("bool(moveit::core::RobotState*, const moveit::core::JointModelGroup*, "
                    "const double*)")


def state_validity_callback(fn, owner=None):
    """
    Wrap a Python ``fn(robot_state, group, values) -> bool`` as MoveIt's
    ``GroupStateValidityCallbackFn`` -- the validity callback ``RobotState::setFromIK``
    calls per IK candidate to make IK **collision-aware**.

    Pass the result as the 4th arg of ``setFromIK(group, pose, timeout, callback)``:
    the C++ solver invokes it for each candidate solution and only accepts one for
    which ``fn`` returns True (e.g. ``not scene.isStateColliding(robot_state,
    group.getName())``). This is a capability **moveit_py cannot express** -- its
    ``set_from_ik(group, pose, timeout)`` has no callback parameter, so it silently
    returns whatever the solver finds, in collision or not.

    Built on ``cppyy_kit.callback`` with the exact ``bool(RobotState*, const
    JointModelGroup*, const double*)`` signature pinned and the lifetime handled
    (``owner`` pins on that object; otherwise process-lifetime). The callback runs in
    the calling C++ thread holding the GIL; a single ``setFromIK`` is single-threaded.
    """
    _ensure_parse()
    return cppyy_kit.callback(fn, signature=_IK_VALIDITY_SIG, owner=owner)


def pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    """An ``Eigen::Isometry3d`` from a translation + (optionally) a quaternion --
    built in C++ because Eigen block assignment (``iso.translation()[i] = v``) is not
    supported across cppyy. Handy for world-collision-object poses and IK targets."""
    _ensure_parse()
    return cppyy.gbl.moveit_kit_cpp.make_pose(
        float(x), float(y), float(z), float(qx), float(qy), float(qz), float(qw))


def warmup(with_kinematics=True, with_planning=False):
    """Front-load moveit_kit's one-time first-use JIT (the pluginlib
    ``createUniqueInstance`` / ``initialize`` call wrappers, ~tens of ms each) during
    init, so the first live ``load_kinematics_solver`` / ``load_planner`` does not
    stall. Builds a throwaway panda model + node and exercises the loaders once."""
    cfg = panda_config()
    from rclcppyy.bringup_rclcpp import bringup_rclcpp
    rclcpp = bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()
    model = build_robot_model(cfg.urdf, cfg.srdf)

    def _kin():
        node = make_node("moveit_kit_warmup_kin")
        load_kinematics_solver(node, model, "panda_arm")

    def _plan():
        node = make_node("moveit_kit_warmup_plan", parameter_overrides(cfg.ompl, "ompl"))
        load_planner(node, model)

    thunks = []
    if with_kinematics:
        _ensure_kinematics()
        thunks.append(_kin)
    if with_planning:
        _ensure_planning()
        thunks.append(_plan)
    cppyy_kit.warmup(*thunks)
