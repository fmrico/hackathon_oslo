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


#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"

#include "visualization_msgs/msg/marker_array.hpp"
#include "bombs_msgs/msg/operate_bomb.hpp"
#include "bombs_msgs/msg/bomb_detection.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

#include "scenario_train/Artificier.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

namespace scenario_train
{

Artificier::Artificier(const rclcpp::NodeOptions & options)
: Node("artificier", options),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  // FIX07 Create Publishers and Subscripcions
  // detection_sub_  = ... ,  std::bind(&Artificier::bomb_detector_callback, this, _1));
  // bomb_operation_pub_ = ...
  // vel_pub_ = ...

  // FIX08 Create Timers
  // timer_ = ...

  // FIX09 Read bombs and codes parameters
  // ...

}

// FIX10 Update last_detections_
// void
// Artificier::bomb_detector_callback(... msg)
// {
//   ...
// }

void
Artificier::control_cycle()
{
  if (last_detections_.empty()) {
    return;
  }

  // FIX11 Select bomb to disable
  const bombs_msgs::msg::BombDetection bomb_to_disable;  // = ... ;

  if (bomb_to_disable.distance < 3.0) {
    disable_bomb(bomb_to_disable);
  }

  if (is_visible(bomb_to_disable)) {
    approach_bomb(bomb_to_disable);
  } else {
    reactive_navigate_bomb(bomb_to_disable);
  }
}

void
Artificier::disable_bomb(const bombs_msgs::msg::BombDetection & bomb)
{
  // FIX12 Send a message to disable a bomb
}

bool
Artificier::is_visible(const bombs_msgs::msg::BombDetection & bomb)
{
  // FIX13 Check if bomb is visible (there is a TF to it)
  // ..  tf_buffer_.lookupTransform(

  return false; // ...(now() - rclcpp::Time(robot2bomb_msg.header.stamp)) < 2s;
}

void
Artificier::approach_bomb(const bombs_msgs::msg::BombDetection & bomb)
{
  // FIX14 send speed commands to the robot to approach the bomb
  // ..  tf_buffer_.lookupTransform(
  // vel_pub_->publish(msg);
}

void
Artificier::reactive_navigate_bomb(const bombs_msgs::msg::BombDetection & bomb)
{
  // FIX15 send speed commands to the robot to find the bomb, only based on distances
  // vel_pub_->publish(msg);
}

}  // namespace scenario_train

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(scenario_train::Artificier)
