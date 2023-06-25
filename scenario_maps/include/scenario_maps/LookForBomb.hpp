// Copyright 2023 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SCENARIO_MAPS__LOOKFORBOMB_HPP_
#define SCENARIO_MAPS__LOOKFORBOMB_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "bombs_msgs/msg/bomb_detection.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace scenario_maps
{

class LookForBomb : public BT::ActionNodeBase
{
public:
  explicit LookForBomb(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

private:
  // FIX29 Add callbacks Subscripcions
  // void bomb_detector_callback(... msg);

  rclcpp::Node::SharedPtr node_;
  // FIX30 Add necessary Publishers and Subscripcions

  std::vector<bombs_msgs::msg::BombDetection> last_detections_;

  // ... Other members you need
};

}  // namespace scenario_maps

#endif  // SCENARIO_MAPS__LOOKFORBOMB_HPP_
