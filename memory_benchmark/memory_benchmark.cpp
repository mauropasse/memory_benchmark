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

using Msg = benchmark_interfaces::msg::Num;
// using Executor = rclcpp::executors::StaticSingleThreadedExecutor;
using Executor = rclcpp::executors::EventsExecutor;


void print_stats(std::string str)
{
  static auto last_memory = 0;
  struct rusage self_usage;
  getrusage(RUSAGE_SELF, &self_usage);
  cout << str << ": " << self_usage.ru_maxrss << " Kb";
  auto diff_memory = self_usage.ru_maxrss - last_memory;
  cout << " - Diff: " << diff_memory << " Kb" << endl;
  last_memory = self_usage.ru_maxrss;
}

class MyNode : public rclcpp::Node
{
public:
  MyNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options)
  : Node(node_name, options)
  {
    auto ipc = rclcpp::IntraProcessSetting::Enable;
    size_t qos_depth = 1;

    if (ipc == rclcpp::IntraProcessSetting::Disable) {
      cout << "Intra process DISABLED (use inter process comm)" << endl;
      cout << "QOS DEPTH: " << qos_depth << endl;
    } else {
      cout << "Intra process ENABLED" << endl;
      cout << "QOS DEPTH: " << qos_depth << endl;
    }
    // Publisher options: avoid creating a QoS event
    auto pub_options = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>();
    pub_options.use_default_callbacks = false;
    pub_options.use_intra_process_comm = ipc;
    publisher_ = create_publisher<Msg>("topic", qos_depth, pub_options);
    print_stats("After create_publisher");

    // Subscriber options: avoid creating a QoS event
    auto sub_options = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>();
    sub_options.use_default_callbacks = false;
    sub_options.use_intra_process_comm = ipc;

    subscription_ = create_subscription<Msg>(
      "topic", qos_depth,
      [this](Msg::SharedPtr msg) {RCLCPP_INFO(get_logger(), "  Got: %ld", msg->num);},
      sub_options);

    print_stats("After create_subscription");

    auto timer_callback = [this]() -> void {
        auto message = std::make_unique<Msg>();
        message->num = count_++;
        RCLCPP_INFO(get_logger(), "Pub: %ld", message->num);
        publisher_->publish(std::move(message));
      };

    timer_ = create_wall_timer(1000ms, timer_callback);
  }

  void publish_loaned()
  {
    print_stats("Before get loaned message");

    std::allocator<void> allocator;
    rclcpp::LoanedMessage<Msg> loaned_msg(*publisher_, allocator);
    print_stats("After get loaned message");

    auto & msg_ref = loaned_msg.get();
    msg_ref.num = 32;
    // std::fill(std::begin(msg_ref.data), std::end(msg_ref.data), 1);
    print_stats("After init loaned message");

    RCLCPP_INFO(get_logger(), "Loaned Pub: %ld", msg_ref.num);

    publisher_->publish(std::move(loaned_msg));
  }

  void publish()
  {
      // Pass message as unique ptr
      print_stats("Before create message");
      // auto message = benchmark_interfaces::msg::Num();
      auto message = std::make_unique<Msg>();
      print_stats("After create message");
      message->num = 32;
      // std::fill(std::begin(message->data), std::end(message->data), 6);
      RCLCPP_INFO(get_logger(), "Pub: %ld", message->num);
      publisher_->publish(std::move(message));
  }

private:
  rclcpp::Subscription<Msg>::SharedPtr subscription_;
  rclcpp::Publisher<Msg>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_{0};
};

int main(int argc, char * argv[])
{
  print_stats("Start test");

  bool use_loan_messages = false;
  auto use_loan = use_loan_messages ? "Using LOANED messages" : "Using UNIQUE_PTR msg (not loaned)";
  cout << use_loan << endl;

  auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
  // Init with disable logging (not implemented in rmw_stub_cpp)
  rclcpp::init(argc, argv, rclcpp::InitOptions().auto_initialize_logging(false));
  print_stats("After rclcpp::init");

  rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
  node_options.use_intra_process_comms(true);
  node_options.start_parameter_services(false);
  node_options.start_parameter_event_publisher(false);

  // Scope
  // {
    cout << endl;
    auto my_node = std::make_shared<MyNode>("my_node", node_options);
    print_stats("After create node");

    auto executor = std::make_shared<Executor>();
    executor->add_node(my_node);

    /* Manual publish
    if (use_loan_messages){
      my_node->publish_loaned();
    } else {
      my_node->publish();
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    print_stats("After publish");
    executor->spin_all(std::chrono::seconds(2));
    */
    std::thread thread([=](){
        executor->spin();
    });
    pthread_setname_np(thread.native_handle(), "Executor thread");
    thread.detach();
    std::this_thread::sleep_for(std::chrono::seconds(5));
  // }

  rclcpp::shutdown();
  print_stats("After shutdown");
}
