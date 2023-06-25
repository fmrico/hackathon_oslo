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


#ifndef BOMBS__BOMB_HPP_
#define BOMBS__BOMB_HPP_

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"

#include "visualization_msgs/msg/marker_array.hpp"
#include "bombs_msgs/msg/operate_bomb.hpp"
#include "bombs_msgs/msg/bomb_detection.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

namespace bombs
{

class Bomb : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Bomb)

  Bomb(
    const int id, const std::string & code, rclcpp::Duration countdown,
    tf2::Stamped<tf2::Transform> init_pos,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void heart_beat();
  double distance_to_bomb();
  void publish_bomb_frame();

  static const int ENABLED = 0;
  static const int DISABLED = 1;
  static const int EXPLODED = 2;

  int get_state() {return bomb_state_;}
  float get_countdown() {return countdown_.seconds();}

protected:
  void publish_markers();
  void bomb_operation_callback(const bombs_msgs::msg::OperateBomb::ConstSharedPtr & msg);

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<bombs_msgs::msg::BombDetection>::SharedPtr detection_pub_;
  rclcpp::Subscription<bombs_msgs::msg::OperateBomb>::SharedPtr bomb_operation_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  bool detected_ {true};
  int id_;
  std::string code_;
  tf2::Stamped<tf2::Transform> global2bomb_;
  rclcpp::Time start_;
  bool timer_started_ {false};
  rclcpp::Duration countdown_;
  double distance_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  int bomb_state_ {ENABLED};
};

}  // namespace bombs

#endif  // BOMBS__BOMB_HPP_
