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

#ifndef SCENARIO_MAPS__ISBOMBDETECTED_HPP_
#define SCENARIO_MAPS__ISBOMBDETECTED_HPP_

#include <string>
#include <vector>
#include <memory>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "bombs_msgs/msg/bomb_detection.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

namespace scenario_maps
{

class IsBombDetected : public BT::ConditionNode
{
public:
  explicit IsBombDetected(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        // FIX20 Create output port for code, bomb_id and bomb_pose
      });
  }

private:
  // FIX26 Add callbacks Subscripcions
  // void bomb_detector_callback(... msg);

  rclcpp::Node::SharedPtr node_;
  // FIX26 Add necessary Publishers and Subscripcions

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string global_frame_ {"map"};


  std::vector<std::string> bombs_;
  std::vector<std::string> codes_;

  std::vector<bombs_msgs::msg::BombDetection> last_detections_;
};

}  // namespace scenario_maps

#endif  // SCENARIO_MAPS__ISBOMBDETECTED_HPP_
