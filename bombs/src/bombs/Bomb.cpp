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


#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"

#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rclcpp/rclcpp.hpp"

#include "bombs/Bomb.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace bombs
{

Bomb::Bomb(
  const int id, const std::string & code, rclcpp::Duration countdown,
  tf2::Stamped<tf2::Transform> init_pos, const rclcpp::NodeOptions & options)
: Node("bomb_" + std::to_string(id), options),
  id_(id),
  code_("bomb_" + std::to_string(id) + "_" + code),
  global2bomb_(init_pos),
  countdown_(countdown),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  RCLCPP_INFO_STREAM(
    get_logger(), "Enabled " << get_name() << " with countdown " << countdown_.seconds());

  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/bombs_markers", 100);
  detection_pub_ = create_publisher<bombs_msgs::msg::BombDetection>("/bomb_detector", 100);
  bomb_operation_sub_ = create_subscription<bombs_msgs::msg::OperateBomb>(
    "/bombs_operation", 10, std::bind(&Bomb::bomb_operation_callback, this, _1));

  timer_ = create_wall_timer(1s, std::bind(&Bomb::heart_beat, this));

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
}

void
Bomb::bomb_operation_callback(const bombs_msgs::msg::OperateBomb::ConstSharedPtr & msg)
{
  if (msg->bomb_id != get_name()) {
    return;
  }

  if (!detected_) {
    return;
  }

  bombs_msgs::msg::BombDetection detection_msg;
  detection_msg.bomb_id = get_name();
  detection_msg.distance = distance_;
  detection_msg.countdown = std::max((countdown_ - (now() - start_)).seconds(), 0.0);

  if (msg->code != code_) {
    RCLCPP_WARN_STREAM(get_logger(), "Code for " << get_name() << " incorrect");
    detection_msg.status = bombs_msgs::msg::BombDetection::EXPLODED;
    detection_pub_->publish(detection_msg);
    bomb_state_ = EXPLODED;
    return;
  }

  if (distance_ > 3.0) {
    RCLCPP_WARN_STREAM(get_logger(), "Operation distance too high " << distance_ << " m.");
  }

  if (msg->operation == bombs_msgs::msg::OperateBomb::ACTIVATE && bomb_state_ == DISABLED) {
    bomb_state_ = ENABLED;
    RCLCPP_INFO_STREAM(get_logger(), "Bomb " << get_name() << " enabled!!!");
  } else if (msg->operation == bombs_msgs::msg::OperateBomb::DEACTIVATE &&
    bomb_state_ == ENABLED)
  {
    detection_msg.status = bombs_msgs::msg::BombDetection::DISABLED;
    detection_pub_->publish(detection_msg);
    bomb_state_ = DISABLED;
    RCLCPP_INFO_STREAM(get_logger(), "Bomb " << get_name() << " disabled!!!");
  }
}

void
Bomb::publish_markers()
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker bomb_marker;
  bomb_marker.header.frame_id = get_name();
  bomb_marker.header.stamp = now() - 10ms;
  bomb_marker.lifetime = rclcpp::Duration(1500ms);
  bomb_marker.id = id_ * 10;
  bomb_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  bomb_marker.action = visualization_msgs::msg::Marker::ADD;
  bomb_marker.scale.x = 0.05;
  bomb_marker.scale.y = 0.05;
  bomb_marker.scale.z = 0.05;
  bomb_marker.scale.x = 0.05;
  switch (bomb_state_) {
    case ENABLED:
      bomb_marker.color.r = 1.0;
      bomb_marker.color.g = 1.0;
      break;
    case DISABLED:
      bomb_marker.color.g = 1.0;
      break;
    case EXPLODED:
      bomb_marker.color.r = 1.0;
      break;
  }

  bomb_marker.color.a = 1.0;
  bomb_marker.mesh_resource = "package://bombs/meshes/bomb.dae";

  markers.markers.push_back(bomb_marker);

  if (bomb_state_ == ENABLED) {
    visualization_msgs::msg::Marker area_1_marker;
    area_1_marker.header.frame_id = get_name();
    area_1_marker.header.stamp = now() - 10ms;
    area_1_marker.lifetime = rclcpp::Duration(1500ms);
    area_1_marker.id = id_ * 10 + 1;
    area_1_marker.type = visualization_msgs::msg::Marker::SPHERE;
    area_1_marker.action = visualization_msgs::msg::Marker::ADD;
    area_1_marker.scale.x = 3.0;
    area_1_marker.scale.y = 3.0;
    area_1_marker.scale.z = 3.0;
    area_1_marker.color.r = 1.0;
    area_1_marker.color.g = 1.0;
    area_1_marker.color.b = 1.0;
    area_1_marker.color.a = 0.3;

    markers.markers.push_back(area_1_marker);

    visualization_msgs::msg::Marker timer_marker;
    timer_marker.header.frame_id = get_name();
    timer_marker.header.stamp = now() - 10ms;
    timer_marker.lifetime = rclcpp::Duration(1500ms);
    timer_marker.id = id_ * 10 + 2;
    timer_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    timer_marker.action = visualization_msgs::msg::Marker::ADD;
    timer_marker.pose.position.z = 1.0;
    timer_marker.scale.x = 0.2;
    timer_marker.scale.y = 0.2;
    timer_marker.scale.z = 0.2;
    timer_marker.color.r = 1.0;
    timer_marker.color.g = 1.0;
    timer_marker.color.b = 1.0;
    timer_marker.color.a = 1.0;
    timer_marker.text = std::to_string(std::max((countdown_ - (now() - start_)).seconds(), 0.0));

    markers.markers.push_back(timer_marker);

  } else if (bomb_state_ == EXPLODED) {
    visualization_msgs::msg::Marker area_1_marker;
    area_1_marker.header.frame_id = get_name();
    area_1_marker.header.stamp = now() - 10ms;
    area_1_marker.lifetime = rclcpp::Duration(1500ms);
    area_1_marker.id = id_ * 10 + 1;
    area_1_marker.type = visualization_msgs::msg::Marker::SPHERE;
    area_1_marker.action = visualization_msgs::msg::Marker::ADD;
    area_1_marker.scale.x = 7.0;
    area_1_marker.scale.y = 7.0;
    area_1_marker.scale.z = 7.0;
    area_1_marker.color.r = 1.0;
    area_1_marker.color.g = 0.0;
    area_1_marker.color.b = 0.0;
    area_1_marker.color.a = 0.9;

    markers.markers.push_back(area_1_marker);
  }
  markers_pub_->publish(markers);
}

double
Bomb::distance_to_bomb()
{
  geometry_msgs::msg::TransformStamped robot2global_msg;
  try {
    robot2global_msg = tf_buffer_.lookupTransform(
      "base_footprint", global2bomb_.frame_id_, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "Bomb position not found while operation: %s", ex.what());
  }
  tf2::Stamped<tf2::Transform> robot2global;
  tf2::fromMsg(robot2global_msg, robot2global);

  tf2::Transform robot_2_bomb = robot2global * global2bomb_;

  const double x = robot_2_bomb.getOrigin().x();
  const double y = robot_2_bomb.getOrigin().y();

  return sqrt(x * x + y * y);
}

void
Bomb::publish_bomb_frame()
{
  geometry_msgs::msg::TransformStamped fixed2bomb_msg;
  fixed2bomb_msg.header.frame_id = global2bomb_.frame_id_;
  fixed2bomb_msg.child_frame_id = get_name();
  fixed2bomb_msg.header.stamp = now();
  fixed2bomb_msg.transform.translation.x = global2bomb_.getOrigin().x();
  fixed2bomb_msg.transform.translation.y = global2bomb_.getOrigin().y();
  fixed2bomb_msg.transform.translation.z = global2bomb_.getOrigin().z();
  tf_broadcaster_->sendTransform(fixed2bomb_msg);
}

void
Bomb::heart_beat()
{
  if (!timer_started_) {
    start_ = now();
    timer_started_ = true;
  }

  if (bomb_state_ != ENABLED) {
    publish_bomb_frame();
    publish_markers();
    return;
  }

  distance_ = distance_to_bomb();
  detected_ = distance_ < 10;

  if (bomb_state_ == ENABLED) {
    bombs_msgs::msg::BombDetection detection_msg;
    detection_msg.bomb_id = get_name();
    detection_msg.distance = distance_;
    detection_msg.countdown = std::max((countdown_ - (now() - start_)).seconds(), 0.0);
    detection_msg.status = bombs_msgs::msg::BombDetection::ENABLED;
    detection_pub_->publish(detection_msg);
  }

  if ((now() - start_) > countdown_ && bomb_state_ == ENABLED) {
    bomb_state_ = EXPLODED;
    bombs_msgs::msg::BombDetection detection_msg;
    detection_msg.bomb_id = get_name();
    detection_msg.distance = distance_;
    detection_msg.countdown = 0.0;
    detection_msg.status = bombs_msgs::msg::BombDetection::EXPLODED;
    detection_pub_->publish(detection_msg);
    RCLCPP_ERROR_STREAM(
      get_logger(), "Bomb " << get_name() << " exploded! [" << (now() - start_).seconds() << "]");
    publish_markers();
  }

  if (detected_ || ((now() - start_) > (countdown_ - 2s))) {
    publish_bomb_frame();
    publish_markers();
  }
}

}  // namespace bombs
