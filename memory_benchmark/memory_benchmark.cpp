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

// Check https://man7.org/linux/man-pages/man2/getrusage.2.html
#include <sys/time.h>
#include <sys/resource.h>
#include <string>
#include <iostream>

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "benchmark_interfaces/msg/num.hpp"

using namespace std::chrono_literals;
using namespace std;

// Adding this class (not instanciate it) don't adds memory overhead
using Msg = benchmark_interfaces::msg::Num;

class MyNode : public rclcpp::Node
{
public:
  MyNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options)
  : Node(node_name, options)
  {
    // Publisher options: avoid creating a QoS event
    auto pub_options = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>();
    pub_options.use_default_callbacks = false;
    pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

    publisher_ = create_publisher<Msg>("topic", 10, pub_options);

    auto timer_callback =
      [this]() -> void {

        // Pub user created Msg
        auto message = benchmark_interfaces::msg::Num();
        message.num = count_++;
        RCLCPP_INFO(get_logger(), "Pub: %ld", message.num);
        publisher_->publish(std::move(message));

        // Pub DDS created Msg
        std::allocator<void> allocator;
        rclcpp::LoanedMessage<Msg> loaned_msg(*publisher_, allocator);
        auto & loaned_msg_ref = loaned_msg.get();
        loaned_msg_ref.num = count_++;
        publisher_->publish(std::move(loaned_msg));

      };

    timer_ = create_wall_timer(1000ms, timer_callback);

    // Subscriber options: avoid creating a QoS event
    auto sub_options = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>();
    sub_options.use_default_callbacks = false;
    sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

    subscription_ = create_subscription<Msg>(
      "topic", 10,
      [this](Msg::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "  Got: %ld", msg->num);
      },
      sub_options);
  }

private:
  rclcpp::Subscription<Msg>::SharedPtr subscription_;
  rclcpp::Publisher<Msg>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_{0};
};


void print_stats(std::string str)
{
  struct rusage self_usage;
  getrusage(RUSAGE_SELF, &self_usage);
  cout << str << ": mem_max_rss_KB: " << self_usage.ru_maxrss << endl;
}

int main(int argc, char * argv[])
{
  // print_stats("Start test");


  // Init with disable logging (not implemented in rmw_stub_cpp)
  rclcpp::init(argc, argv, rclcpp::InitOptions().auto_initialize_logging(false));
  print_stats("After rclcpp::init");
#if 0

  rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
  node_options.use_intra_process_comms(true);
  node_options.start_parameter_services(false);
  node_options.start_parameter_event_publisher(false);

  print_stats("After rclcpp::NodeOptions");

  auto my_node = std::make_shared<MyNode>("my_node", node_options);

  print_stats("After create node");

  auto executor = std::make_shared<rclcpp::executors::EventsExecutor>();

  print_stats("After create executor");

  executor->add_node(my_node);

  std::thread thread([=](){ executor->spin(); });
  pthread_setname_np(thread.native_handle(), "Executor");
  thread.detach();

  std::this_thread::sleep_for(std::chrono::seconds(3));

  print_stats("Before shutdown");
  rclcpp::shutdown();

# endif
  // std::this_thread::sleep_for(std::chrono::seconds(30));
  // print_stats("End test");
  int i;
  cin >> i;
  cout << "Enter value to continue: " << endl;
  print_stats("Start test");
}
