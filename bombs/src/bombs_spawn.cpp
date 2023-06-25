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


#include "tf2/transform_datatypes.h"

#include "rclcpp/rclcpp.hpp"

#include "bombs/Bomb.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  tf2::Transform pos_bomb_0(tf2::Quaternion(), tf2::Vector3(1.0, 0.0, 0.0));
  tf2::Stamped<tf2::Transform> pos_bomb_0_stamped(pos_bomb_0, tf2::TimePointZero, "odom");

  auto bomb_0_node = bombs::Bomb::make_shared(0, "1234", 50s, pos_bomb_0_stamped);
  rclcpp::spin(bomb_0_node);

  rclcpp::shutdown();
  return 0;
}
