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

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using LaserScan = sensor_msgs::msg::LaserScan;

class LaserSub : public rclcpp::Node {
private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_sub;

public:
  LaserSub() : Node("topic_sub_oop_node") {
    // important!! make subscriber
    m_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "skidbot/scan", 10,
        // sub_callback must have one or more parameter, because parameters are datas of subscribe
        std::bind(&LaserSub::sub_callback, this, std::placeholders::_1));
  }
  // SharedPtr is smart pointer
  void sub_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // ranges 배열이 기존 Python에서는 list 형식으로 표현되었지만, C++에서는 vector로 표현된다는 점에서 차이가 있습니다.
    // std::cout << (msg->ranges).size() << std::endl;

    // for (auto e : msg->ranges)
    //   std::cout << e << std::endl;
    // msg->ranges : 0 ~ 720, 360 : front 
    RCLCPP_INFO(this->get_logger(), "Distance from Front Object : %f", (msg->ranges)[360]);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // subscriber는 반드시 spin을 통한 갱신이 있어야 최신 message를 수신할 수 있다는 점을 다시 한 번 말씀드립니다.
  rclcpp::spin(std::make_shared<LaserSub>());
  rclcpp::shutdown();

  return 0;
}
