// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "benchmark_interfaces/msg/num.hpp"

using namespace std::chrono_literals;

/*
   Description: <wip>
*/

class MyNode : public rclcpp::Node
{
public:
  MyNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options)
  : Node(node_name, options)
  {
    // Publisher stuff
    publisher_ = create_publisher<benchmark_interfaces::msg::Num>("topic", 10);

    auto timer_callback =
      [this]() -> void {
        auto message = benchmark_interfaces::msg::Num();
        message.num = count_++;
        RCLCPP_INFO(get_logger(), "Pub: %ld", message.num);
        publisher_->publish(message);
      };
    timer_ = create_wall_timer(500ms, timer_callback);

    // Subscriber stuff
    subscription_ = create_subscription<benchmark_interfaces::msg::Num>(
      "topic",
      10,
      [this](benchmark_interfaces::msg::Num::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "   Got: %ld", msg->num);
      });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<benchmark_interfaces::msg::Num>::SharedPtr publisher_;
  rclcpp::Subscription<benchmark_interfaces::msg::Num>::SharedPtr subscription_;
  size_t count_{0};
};

int main(int argc, char * argv[])
{
  std::cout << "Start test\n" << std::endl;

  // Init with disable logging (not implemented in rmw_stub_cpp)
  rclcpp::init(argc, argv, rclcpp::InitOptions().auto_initialize_logging(false));

  rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
  node_options.use_intra_process_comms(true);
  node_options.start_parameter_services(false);
  node_options.start_parameter_event_publisher(false);

  auto my_node = std::make_shared<MyNode>("my_node", node_options);
  auto executor = std::make_shared<rclcpp::executors::EventsExecutor>();

  executor->add_node(my_node);

  std::thread thread([=](){ executor->spin(); });
  pthread_setname_np(thread.native_handle(), "Executor");
  thread.detach();

  std::this_thread::sleep_for(std::chrono::seconds(3));

  rclcpp::shutdown();

  std::cout << "\nEnd test" << std::endl;
}
