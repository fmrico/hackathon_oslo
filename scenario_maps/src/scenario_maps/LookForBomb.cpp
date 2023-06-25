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

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "bombs_msgs/msg/bomb_detection.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

#include "scenario_maps/LookForBomb.hpp"

namespace scenario_maps
{

using namespace std::chrono_literals;
using std::placeholders::_1;

LookForBomb::LookForBomb(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  // FIX47 Create necessary Publishers and Subscripcions
}

void
LookForBomb::halt()
{
}

// FIX43 Update last_detections_
// void
// LookForBomb::bomb_detector_callback(... msg)
// {
//   ...
// }

BT::NodeStatus
LookForBomb::tick()
{
  // FIX45 send speed commands to the robot to find the bomb, only based on distances
  // vel_pub_->publish(msg);

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "LookForBomb RUNNING");
  return BT::NodeStatus::RUNNING;
}

}  // namespace scenario_maps

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<scenario_maps::LookForBomb>("LookForBomb");
}
