// L2 lowering of t01's ApproachObject leaf: the Python callback becomes a native
// C++ SyncActionNode, compiled into a BehaviorTree.CPP plugin .so and registered
// via BehaviorTreeFactory::registerFromPlugin() -- the engine dlopen()s this
// library and calls BT_RegisterNodesFromPlugin below. No cppyy, no JIT, no Python
// callback per tick: the leaf executes as native code at engine speed.
//
// Build: scripts/freeze/build_l2_node.py (or `pixi run -e bt freeze-l2-build`).
//
// Two nodes are registered:
//   ApproachObject        -- prints, for the correctness differential vs the
//                            Python leaf (identical stdout, identical status);
//   ApproachObjectSilent  -- no I/O, for the tick-rate differential (isolates the
//                            per-leaf boundary cost: native vs Python-through-kit).
#include <iostream>

#include "behaviortree_cpp/bt_factory.h"

using BT::NodeStatus;
using BT::NodeConfig;
using BT::SyncActionNode;

class ApproachObject : public SyncActionNode
{
public:
  ApproachObject(const std::string& name, const NodeConfig& config)
    : SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() { return {}; }
  NodeStatus tick() override
  {
    std::cout << "ApproachObject: approach_object" << std::endl;
    return NodeStatus::SUCCESS;
  }
};

class ApproachObjectSilent : public SyncActionNode
{
public:
  ApproachObjectSilent(const std::string& name, const NodeConfig& config)
    : SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() { return {}; }
  NodeStatus tick() override { return NodeStatus::SUCCESS; }
};

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ApproachObject>("ApproachObject");
  factory.registerNodeType<ApproachObjectSilent>("ApproachObjectSilent");
}
