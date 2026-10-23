#ifndef SYNCAI_COSTMAP_2D__MAP_TYPE_GATE_HPP_
#define SYNCAI_COSTMAP_2D__MAP_TYPE_GATE_HPP_

// Which map type a layer runs on (2026-10-20, feature/octomap-planner-3d).
// The local costmap loads two sensor layers, ObstacleLayer for a 2D map and
// TerrainLayer for a 3D one, and exactly one of them may run at a time: they
// disagree on a ramp (free in the terrain layer, a wall in the obstacle
// layer's height band) and the max of the two is the wall. A map is 2D or 3D,
// never both (2026-10-19), and the planner's Octo3D says which on its latched
// `map_available` -- the same signal, on the same topic, that the
// task_runner picks the *_3d.xml trees by, so the costmap and the tree switch
// together. Per update, not once at boot, because Octo3D/load_map switches
// maps without restarting the controller.
//
// A layer's `map_type` parameter: "any" (the default -- the gate is open and
// subscribes to nothing, which is every layer of the global costmap), "2d" or
// "3d". Until the first message the map reads 2D, as it does for the
// task_runner: a goal in the first moments after boot runs the 2D tree on the
// 2D layer.
//
// The gate does not touch the layer's `enabled_`: that stays the operator's
// (the `enabled` parameter, ObstacleLayer's dynamic one included). A layer
// asks active() in its updateBounds() / updateCosts(), and changed() once a
// cycle, to repaint its whole window when it goes on or off -- see
// ObstacleLayer::updateBounds().

#include <atomic>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace syncai_costmap_2d
{

class MapTypeGate
{
public:
  // `map_type` is the layer's parameter ("any", "2d", "3d"; anything else
  // throws), `map_3d_topic` the resolved name of Octo3D's map_available.
  // `layer_name` is for the log line on a switch.
  void initialize(
    const rclcpp::Node::SharedPtr & node, const std::string & layer_name,
    const std::string & map_type, const std::string & map_3d_topic,
    const rclcpp::CallbackGroup::SharedPtr & callback_group);

  // Whether the layer runs on the current map. Always true for "any".
  bool active() const;

  // True once after active() has changed (and on the first call, so a layer
  // that starts gated off repaints nothing it never painted -- harmless).
  // Call it from the costmap update thread only.
  bool changed();

  bool gated() const { return want_ != Want::Any; }

private:
  enum class Want { Any, Map2D, Map3D };
  Want want_{Want::Any};
  std::atomic<bool> map_3d_{false};
  bool last_active_{true};
  bool first_{true};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
};

}  // namespace syncai_costmap_2d

#endif  // SYNCAI_COSTMAP_2D__MAP_TYPE_GATE_HPP_
