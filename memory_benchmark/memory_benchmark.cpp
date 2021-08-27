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

#include "benchmark_interfaces/msg/num0.hpp"
#include "benchmark_interfaces/msg/num1.hpp"
#include "benchmark_interfaces/msg/num2.hpp"

using namespace std::chrono_literals;
using namespace std;

using Executor = rclcpp::executors::StaticSingleThreadedExecutor;
// using Executor = rclcpp::executors::SingleThreadedExecutor;
// using Executor = rclcpp::executors::EventsExecutor;

size_t count_{0};

auto start = std::chrono::high_resolution_clock::now();
auto finish = std::chrono::high_resolution_clock::now();

void print_stats(std::string str)
{
  static auto last_memory = 0;
  struct rusage self_usage;
  getrusage(RUSAGE_SELF, &self_usage);
  auto diff_memory = self_usage.ru_maxrss - last_memory;
  cout << str << ": " << self_usage.ru_maxrss << " Kb - Diff: " << diff_memory << " Kb" << endl;
  last_memory = self_usage.ru_maxrss;
}

void start_int()
{
  start = std::chrono::high_resolution_clock::now();
  // uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::
  //                 now().time_since_epoch()).count();
  // std::cout <<  us << " A ";
}
void end_int()
{
  finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::micro> elapsed = finish - start;
  cout << elapsed.count() << " ";

  // uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::
  //                 now().time_since_epoch()).count();
  // std::cout <<  us << " Z ";
}

#define CREATE_PUB_SUB(name, msg_name) \
  publisher_ ## name = create_publisher< benchmark_interfaces::msg::Num ## msg_name>( \
    "topic_" #name, qos_depth, pub_options); \
  subscription_ ## name = create_subscription< benchmark_interfaces::msg::Num ## msg_name>( \
    "topic_" #name, qos_depth, \
    [this](benchmark_interfaces::msg::Num ## msg_name::SharedPtr msg) { \
      RCLCPP_INFO(get_logger(), # name " got: %d", msg->data.size()); \
    }, sub_options);


#define DECLARE_PUB_SUB(name, msg_name) \
  rclcpp::Publisher<benchmark_interfaces::msg::Num ## msg_name>::SharedPtr publisher_ ## name; \
  rclcpp::Subscription<benchmark_interfaces::msg::Num ## msg_name>::SharedPtr subscription_ ## name;

class MyNode : public rclcpp::Node
{
public:
  MyNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options)
  : Node(node_name, options)
  {
    static size_t num_nodes_{1};
    cout << "Created: " << num_nodes_++ << " nodes." << endl;
    // auto ipc = rclcpp::IntraProcessSetting::Disable;
    size_t qos_depth = 1;

    // Publisher options: avoid creating a QoS event
    auto pub_options = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>();
    pub_options.use_default_callbacks = false;
    pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

    // Subscriber options: avoid creating a QoS event
    auto sub_options = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>();
    sub_options.use_default_callbacks = false;
    sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

    // CREATE_PUB_SUB(0,0)
    publisher_0 = create_publisher< benchmark_interfaces::msg::Num0>("topic_0", qos_depth, pub_options);
    // subscription_0 = create_subscription< benchmark_interfaces::msg::Num0>(
    //   "topic_0", qos_depth,
    //   [this](benchmark_interfaces::msg::Num0::SharedPtr msg) {
    //     // RCLCPP_INFO(get_logger(), "topic_0 got: %ld", msg->data.size());
    //   }, sub_options);

    // CREATE_PUB_SUB(1,0)
    pub_options.use_default_callbacks = false;
    pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    sub_options.use_default_callbacks = false;
    sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;

    publisher_1 = create_publisher< benchmark_interfaces::msg::Num1>("topic_1", qos_depth, pub_options);
    // subscription_1 = create_subscription< benchmark_interfaces::msg::Num1>(
    //   "topic_1", qos_depth,
    //   [this](benchmark_interfaces::msg::Num1::SharedPtr msg) {
    //     // RCLCPP_INFO(get_logger(), "topic_1 got: %ld", msg->data.size());
    //   }, sub_options);

    publisher_2 = create_publisher< benchmark_interfaces::msg::Num2>("topic_2", qos_depth, pub_options);
    // subscription_2 = create_subscription< benchmark_interfaces::msg::Num2>(
    //   "topic_2", qos_depth,
    //   [this](benchmark_interfaces::msg::Num2::SharedPtr msg) {
    //     // RCLCPP_INFO(get_logger(), "topic_0 got: %ld", msg->data.size());
    //   }, sub_options);
  }

#define PUBLISH(name, msg_name) \
  auto message ## name = std::make_unique<benchmark_interfaces::msg::Num ## msg_name>(); \
  publisher_ ## name->publish(std::move(message ## name));


#define PUBLISH_LOANED(name, msg_name) \
  std::allocator<void> allocator; \
  rclcpp::LoanedMessage<benchmark_interfaces::msg::Num ## msg_name> loaned_msg(*publisher_ ## name, allocator); \
  publisher_ ## name->publish(std::move(loaned_msg));

  // auto & msg_ref = loaned_msg.get();
  // msg_ref.num = 32;
  // std::fill(std::begin(msg_ref.data), std::end(msg_ref.data), 33);

  void publish_messages()
  {
    // std::this_thread::sleep_for(1ms);
    PUBLISH(0,0)
    // std::this_thread::sleep_for(1ms);
    PUBLISH_LOANED(1,1)
    // std::this_thread::sleep_for(1ms);
    PUBLISH(2,2)
    // cout << endl;
  }

private:
  DECLARE_PUB_SUB(0,0)
  DECLARE_PUB_SUB(1,1)
  DECLARE_PUB_SUB(2,2)
};


int main(int argc, char * argv[])
{
  print_stats("Start test");

  auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
  // Init with disable logging (not implemented in rmw_stub_cpp)
  rclcpp::init(argc, argv, rclcpp::InitOptions().auto_initialize_logging(false));
  print_stats("After rclcpp::init");

  /******** CREATE NODE ********************************************************************/
  rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
  node_options.use_intra_process_comms(true);
  node_options.start_parameter_services(false);
  node_options.start_parameter_event_publisher(false);

  auto my_node0 = std::make_shared<MyNode>("my_node0", node_options);

  print_stats("After create node");
  /*********************************************************************************************/

  /******** EXECUTOR THREAD ********************************************************************/
  // auto executor = std::make_shared<Executor>();
  // executor->add_node(my_node0);

  // std::thread spin_thread([=](){
  //     executor->spin();
  //     // while(true) {executor->spin_some();std::this_thread::sleep_for(1s);}
  // });
  // pthread_setname_np(spin_thread.native_handle(), "Executor thread");
  // struct sched_param spin_thread_param;
  // spin_thread_param.sched_priority = sched_get_priority_min(SCHED_FIFO);
  // pthread_setschedparam(spin_thread.native_handle(), SCHED_FIFO, &spin_thread_param);
  // spin_thread.detach();
  /*********************************************************************************************/

  /******** PUBLISHER THREAD ********************************************************************/
  std::thread pub_thread([=](){
      while(true) {
        my_node0->publish_messages();
        // std::this_thread::sleep_for(2ms);
      }
  });
  pthread_setname_np(pub_thread.native_handle(), "Pub thread");
  struct sched_param pub_thread_param;
  pub_thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO);
  pthread_setschedparam(pub_thread.native_handle(), SCHED_FIFO, &pub_thread_param);
  pub_thread.detach();
  /*********************************************************************************************/

  // std::this_thread::sleep_for(1s);
  int i;
  cin >> i;
  rclcpp::shutdown();
  print_stats("After shutdown");
}

