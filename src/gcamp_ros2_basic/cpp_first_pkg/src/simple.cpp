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

// ros2 c++ api include
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  
  // init rclcpp for ros2 configuration
  rclcpp::init(argc, argv);

  // recommend using make_shared("node name") for making node
  // auto is 'deduction' real type is 'std::shared_ptr<rclcpp::Node>'
  auto node = rclcpp::Node::make_shared("simple_node");

  // get_logger() exist in node
  RCLCPP_INFO(node->get_logger(), "Logger Test");

  // close ros2 
  rclcpp::shutdown();
  return 0;
}
