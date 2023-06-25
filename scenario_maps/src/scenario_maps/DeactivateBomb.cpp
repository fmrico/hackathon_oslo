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

#include <string>
#include <iostream>

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "bombs_msgs/msg/operate_bomb.hpp"

#include "rclcpp/rclcpp.hpp"

#include "scenario_maps/DeactivateBomb.hpp"

namespace scenario_maps
{

DeactivateBomb::DeactivateBomb(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  // FIX31 Create necessary Publishers and Subscripcions
}

void
DeactivateBomb::halt()
{
}

BT::NodeStatus
DeactivateBomb::tick()
{
  // FIX32 Read bomb_id and code from input port
  // ...
  // getInput("...", ...);

  // FIX33 Send a message to disable a bomb

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "DeactivateBomb SUCCESS");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace scenario_maps

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<scenario_maps::DeactivateBomb>("DeactivateBomb");
}
