// Copyright 2021 Seoul Business Agency Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// [http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("simple_loop_node");

  // WallRate & Rate
  // https://qiita.com/NeK/items/bcf518f6dd79f970bb8e
  // rclcpp::WallRate is timmer in ros2 (the other timmer is rclcpp::Rate)
  // recommend rclcpp::WallRate
  // make rate instance, parameter is period
  rclcpp::WallRate rate(2);  // Hz

  // similar to arduino loop function(periodically run node)
  while (rclcpp::ok()) {
    RCLCPP_INFO(node->get_logger(), "Simple Loop Node");
    // this part update node
    // similar to python spin_once
    rclcpp::spin_some(node);
    // similar to python time.sleep
    // repeat while loop 2Hz
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
