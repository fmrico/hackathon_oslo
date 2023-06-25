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

#include <random>

#include "tf2/transform_datatypes.h"

#include "rclcpp/rclcpp.hpp"

#include "bombs/Bomb.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_temp = rclcpp::Node::make_shared("artificier");
  std::vector<std::string> bombs;
  std::vector<std::string> codes;
  node_temp->declare_parameter("bombs", bombs);
  node_temp->declare_parameter("codes", codes);
  node_temp->get_parameter("bombs", bombs);
  node_temp->get_parameter("codes", codes);
  node_temp = nullptr;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> pos_distrib(-20, 20);
  std::uniform_int_distribution<> time_ditrib(100, 200);

  rclcpp::executors::SingleThreadedExecutor exe;

  std::list<bombs::Bomb::SharedPtr> bomb_list_static;
  for (size_t i = 0; i < bombs.size(); i++) {
    rclcpp::Duration countdown(time_ditrib(gen), 0);
    tf2::Vector3 v3pos(pos_distrib(gen), pos_distrib(gen), 0.0);
    tf2::Transform pos_bomb(tf2::Quaternion(), v3pos);
    tf2::Stamped<tf2::Transform> pos_bomb_stamped(pos_bomb, tf2::TimePointZero, "odom");

    auto bomb_node = bombs::Bomb::make_shared(i, codes[i], countdown, pos_bomb_stamped);
    bomb_list_static.push_back(bomb_node);
    exe.add_node(bomb_node);
  }

  float points = 0.0;

  std::list<bombs::Bomb::SharedPtr> bomb_list = bomb_list_static;

  rclcpp::Rate rate(50);
  while (rclcpp::ok() && !bomb_list.empty()) {
    auto it = bomb_list.begin();
    while (it != bomb_list.end()) {
      if ((*it)->get_state() != bombs::Bomb::ENABLED) {
        if ((*it)->get_state() == bombs::Bomb::EXPLODED) {
          points += -500.0;
        } else {
          points += (*it)->get_countdown();
        }
        std::cerr << "Points: " << points << std::endl;
        // exe.remove_node(*it);
        it = bomb_list.erase(it);
      } else {
        ++it;
      }
    }

    exe.spin_some();
    rate.sleep();
  }

  std::cerr << "Final points: " << points << std::endl;


  rclcpp::shutdown();
  return 0;
}
