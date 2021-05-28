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

#include "benchmark_interfaces/msg/num_a.hpp"
#include "benchmark_interfaces/msg/num_b.hpp"
#include "benchmark_interfaces/msg/num_c.hpp"
#include "benchmark_interfaces/msg/num_d.hpp"
#include "benchmark_interfaces/msg/num_e.hpp"

using namespace std::chrono_literals;
using namespace std;

using MsgA = benchmark_interfaces::msg::NumA;
using MsgB = benchmark_interfaces::msg::NumB;
using MsgC = benchmark_interfaces::msg::NumC;
using MsgD = benchmark_interfaces::msg::NumD;
using MsgE = benchmark_interfaces::msg::NumE;
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


auto start = std::chrono::high_resolution_clock::now();
auto finish = std::chrono::high_resolution_clock::now();

void start_int()
{
  start = std::chrono::high_resolution_clock::now();
}

void end_int()
{
  finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::micro> elapsed = finish - start;
  cout << elapsed.count() << " ";
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
    publisher_A = create_publisher<MsgA>("topic_A", qos_depth, pub_options);
    publisher_B = create_publisher<MsgB>("topic_B", qos_depth, pub_options);
    publisher_C = create_publisher<MsgC>("topic_C", qos_depth, pub_options);
    publisher_D = create_publisher<MsgD>("topic_D", qos_depth, pub_options);
    publisher_E = create_publisher<MsgE>("topic_E", qos_depth, pub_options);
    print_stats("After create_publisher");

    // Subscriber options: avoid creating a QoS event
    auto sub_options = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>();
    sub_options.use_default_callbacks = false;
    sub_options.use_intra_process_comm = ipc;

    subscription_A = create_subscription<MsgA>(
      "topic_A", qos_depth,
      [this](MsgA::SharedPtr msg) {
        // std::this_thread::sleep_for(std::chrono::microseconds(100));
        // RCLCPP_INFO(get_logger(), " A Got: %ld", msg->num);
      },
      sub_options);

    subscription_B = create_subscription<MsgB>(
      "topic_A", qos_depth,
      [this](MsgB::SharedPtr msg) {
        // std::this_thread::sleep_for(std::chrono::microseconds(100));
      },
      sub_options);

    subscription_C = create_subscription<MsgC>(
      "topic_C", qos_depth,
      [this](MsgC::SharedPtr msg) {
        // std::this_thread::sleep_for(std::chrono::microseconds(100));
      },
      sub_options);

    subscription_D = create_subscription<MsgD>(
      "topic_D", qos_depth,
      [this](MsgD::SharedPtr msg) {
        // std::this_thread::sleep_for(std::chrono::microseconds(100));
      },
      sub_options);

    subscription_E = create_subscription<MsgE>(
      "topic_E", qos_depth,
      [this](MsgE::SharedPtr msg) {
        // std::this_thread::sleep_for(std::chrono::microseconds(100));
      },
      sub_options);

    print_stats("After create_subscription");
  }

  void publish_messages()
  {

    auto messageB = std::make_unique<MsgB>();
    messageB->num = count_;
    start_int();
    publisher_B->publish(std::move(messageB));
    end_int();

    auto messageC = std::make_unique<MsgC>();
    messageC->num = count_;
    start_int();
    publisher_C->publish(std::move(messageC));
    end_int();

    auto messageD = std::make_unique<MsgD>();
    messageD->num = count_;
    start_int();
    publisher_D->publish(std::move(messageD));
    end_int();

    auto messageE = std::make_unique<MsgE>();
    messageE->num = count_;
    start_int();
    publisher_E->publish(std::move(messageE));
    end_int();

    auto messageA = std::make_unique<MsgA>();
    messageA->num = count_;
    start_int();
    publisher_A->publish(std::move(messageA));
    end_int();

    cout << endl;
    count_++;
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }

  void publish_loaned()
  {
    print_stats("Before get loaned message");

    std::allocator<void> allocator;
    rclcpp::LoanedMessage<MsgA> loaned_msg(*publisher_A, allocator);
    print_stats("After get loaned message");

    auto & msg_ref = loaned_msg.get();
    msg_ref.num = 32;
    // std::fill(std::begin(msg_ref.data), std::end(msg_ref.data), 1);
    print_stats("After init loaned message");

    // RCLCPP_INFO(get_logger(), "Loaned Pub: %ld", msg_ref.num);

    publisher_A->publish(std::move(loaned_msg));
  }

  void publish()
  {
      // Pass message as unique ptr
      print_stats("Before create message");
      // auto message = benchmark_interfaces::msg::NumA();
      auto message = std::make_unique<MsgA>();
      print_stats("After create message");
      message->num = 32;
      // std::fill(std::begin(message->data), std::end(message->data), 6);
      // RCLCPP_INFO(get_logger(), "Pub: %ld", message->num);
      publisher_A->publish(std::move(message));
  }

private:
  rclcpp::Subscription<MsgA>::SharedPtr subscription_A;
  rclcpp::Publisher<MsgA>::SharedPtr publisher_A;

  rclcpp::Subscription<MsgB>::SharedPtr subscription_B;
  rclcpp::Publisher<MsgB>::SharedPtr publisher_B;

  rclcpp::Subscription<MsgC>::SharedPtr subscription_C;
  rclcpp::Publisher<MsgC>::SharedPtr publisher_C;

  rclcpp::Subscription<MsgD>::SharedPtr subscription_D;
  rclcpp::Publisher<MsgD>::SharedPtr publisher_D;

  rclcpp::Subscription<MsgE>::SharedPtr subscription_E;
  rclcpp::Publisher<MsgE>::SharedPtr publisher_E;

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

  cout << endl;
  auto my_node = std::make_shared<MyNode>("my_node", node_options);
  print_stats("After create node");

  auto executor = std::make_shared<Executor>();
  executor->add_node(my_node);


  // Create SPIN thread
  std::thread spin_thread([=](){
      executor->spin();
      // while(true) {
      //   executor->spin_some();
      //   std::this_thread::sleep_for(std::chrono::milliseconds(4));
      // }
  });
  pthread_setname_np(spin_thread.native_handle(), "Executor thread");
  struct sched_param spin_thread_param;
  spin_thread_param.sched_priority = sched_get_priority_min(SCHED_FIFO);
  pthread_setschedparam(spin_thread.native_handle(), SCHED_FIFO, &spin_thread_param);
  spin_thread.detach();


  // Create Publisher thread
  std::thread pub_thread([=](){
      while(true) {
        my_node->publish_messages();
      }
  });
  pthread_setname_np(pub_thread.native_handle(), "Pub thread");

  struct sched_param pub_thread_param;
  pub_thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO);
  pthread_setschedparam(pub_thread.native_handle(), SCHED_FIFO, &pub_thread_param);
  pub_thread.detach();

  std::this_thread::sleep_for(std::chrono::seconds(3));

  rclcpp::shutdown();
  print_stats("After shutdown");
}
