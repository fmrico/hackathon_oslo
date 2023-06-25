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

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

#include "scenario_maps/IsBombDetected.hpp"

namespace scenario_maps
{

using namespace std::chrono_literals;
using namespace std::placeholders;

IsBombDetected::IsBombDetected(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);

  // FIX34 Create necessary Publishers and Subscripcions

  // FIX35 Read bombs and codes parameters
  // ...
  // don't forget global_frame, in case we are not in map frame
}

// FIX36 Update last_detections_
// void
// IsBombDetected::bomb_detector_callback(const bombs_msgs::msg::BombDetection & msg)
// {
//   ...
// }


BT::NodeStatus
IsBombDetected::tick()
{
  const bombs_msgs::msg::BombDetection bomb_to_disable;  // = ... ;

  // FIX37 Update last_detections_

  // FIX38 Check if bomb is visible (there is a TF to it)
  // ..  tf_buffer_.lookupTransform(

  // FIX39 if visible
  //  - Set ouput ports with bomb_id, code, and pose
  //  - return BT::NodeStatus::SUCCESS
  // Otherwise return BT::NodeStatus::FAILURE

  // setOutput("...", ...);

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "IsBombDetected SUCCESS");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace scenario_maps

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<scenario_maps::IsBombDetected>("IsBombDetected");
}
