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
// important!! Talker class inherit rclcpp::Node
class Talker : public rclcpp::Node {
private:
  // make timmer for periodically running node  
  rclcpp::TimerBase::SharedPtr m_timer;
  size_t m_count;

  void timer_callback() {
    m_count++;
    RCLCPP_INFO(this->get_logger(), "I am Simple OOP Example, count : %d",
                m_count);
  }

public:
  // if you construct node, you should set name
  Talker() : Node("simple_oop_node") {
    // create_wall_timer 함수에 timer와 실행시킬 함수를 전달하면 편리하게 주기적 실행을 할 수 있습니다.
		// this->는 굳이 명시하지 않아도 됩니다.
    m_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                      std::bind(&Talker::timer_callback, this));
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // spin은 Node내부에 정해진 timer에 따라 Node를 주기적으로 동작, 갱신시켜줍니다.
  dsddsdsdsdfdsdfsd
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}
