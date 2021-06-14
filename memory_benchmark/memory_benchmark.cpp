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

#include "benchmark_interfaces/msg/num_f.hpp"
#include "benchmark_interfaces/msg/num_g.hpp"
#include "benchmark_interfaces/msg/num_h.hpp"
#include "benchmark_interfaces/msg/num_i.hpp"
#include "benchmark_interfaces/msg/num_j.hpp"

using namespace std::chrono_literals;
using namespace std;

using Executor = rclcpp::executors::StaticSingleThreadedExecutor;
// using Executor = rclcpp::executors::SingleThreadedExecutor;
// using Executor = rclcpp::executors::EventsExecutor;

auto start = std::chrono::high_resolution_clock::now();
auto finish = std::chrono::high_resolution_clock::now();

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

void start_int()
{
  // start = std::chrono::high_resolution_clock::now();
}
void end_int()
{
  // finish = std::chrono::high_resolution_clock::now();
  // std::chrono::duration<double, std::micro> elapsed = finish - start;
  // cout << elapsed.count() << " ";
}

#define CREATE_PUB_SUB(name, msg_name) \
  publisher_ ## name = create_publisher< benchmark_interfaces::msg::Num ## msg_name>( \
    "topic_" #name, qos_depth, pub_options); \
  subscription_ ## name = create_subscription< benchmark_interfaces::msg::Num ## msg_name>( \
    "topic_" #name, qos_depth, \
    [this](benchmark_interfaces::msg::Num ## msg_name::SharedPtr msg) { \
      RCLCPP_INFO(get_logger(), # name " got: %lld", msg->num); \
    }, sub_options);

#define DECLARE_PUB_SUB(name, msg_name) \
  rclcpp::Subscription<benchmark_interfaces::msg::Num ## msg_name>::SharedPtr subscription_ ## name; \
  rclcpp::Publisher<benchmark_interfaces::msg::Num ## msg_name>::SharedPtr publisher_ ## name;

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

    // Publisher options: avoid creating a QoS event
    auto pub_options = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>();
    pub_options.use_default_callbacks = false;
    pub_options.use_intra_process_comm = ipc;

    // Subscriber options: avoid creating a QoS event
    auto sub_options = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>();
    sub_options.use_default_callbacks = false;
    sub_options.use_intra_process_comm = ipc;

    CREATE_PUB_SUB(1, A)
    CREATE_PUB_SUB(2, B)
    CREATE_PUB_SUB(3, C)
    CREATE_PUB_SUB(4, D)
    CREATE_PUB_SUB(5, E)
    CREATE_PUB_SUB(6, F)
    CREATE_PUB_SUB(7, G)
    CREATE_PUB_SUB(8, H)
    CREATE_PUB_SUB(9, I)
    CREATE_PUB_SUB(10, J)
    print_stats("After create publishers and subscribers");
  }

#define PUBLISH(name, msg_name) \
  auto message ## name = std::make_unique<benchmark_interfaces::msg::Num ## msg_name>(); \
  message ## name->num = count_; \
  publisher_ ## name->publish(std::move(message ## name));

  void publish_messages()
  {
    PUBLISH(1, A)
    PUBLISH(2, B)
    PUBLISH(3, C)
    PUBLISH(4, D)
    PUBLISH(5, E)
    PUBLISH(6, F)
    PUBLISH(7, G)
    PUBLISH(8, H)
    PUBLISH(9, I)
    PUBLISH(10, J)
    count_++; cout << endl;
    std::this_thread::sleep_for(1s);
  }

private:
  DECLARE_PUB_SUB(1, A)
  DECLARE_PUB_SUB(2, B)
  DECLARE_PUB_SUB(3, C)
  DECLARE_PUB_SUB(4, D)
  DECLARE_PUB_SUB(5, E)
  DECLARE_PUB_SUB(6, F)
  DECLARE_PUB_SUB(7, G)
  DECLARE_PUB_SUB(8, H)
  DECLARE_PUB_SUB(9, I)
  DECLARE_PUB_SUB(10, J)
  size_t count_{0};
};

int main(int argc, char * argv[])
{
  print_stats("Start test");

  auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
  // Init with disable logging (not implemented in rmw_stub_cpp)
  rclcpp::init(argc, argv, rclcpp::InitOptions().auto_initialize_logging(false));
  print_stats("After rclcpp::init");

  rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
  node_options.use_intra_process_comms(true);
  node_options.start_parameter_services(false);
  node_options.start_parameter_event_publisher(false);

  /******** CREATE NODE ********************************************************************/
  auto my_node = std::make_shared<MyNode>("my_node", node_options);
  print_stats("After create node");
  std::this_thread::sleep_for(3s);
  /*********************************************************************************************/

  /******** EXECUTOR THREAD ********************************************************************/
  auto executor = std::make_shared<Executor>();
  executor->add_node(my_node);
  std::thread spin_thread([=](){
      executor->spin();
      // while(true) {executor->spin_some();std::this_thread::sleep_for(1s);}
  });
  pthread_setname_np(spin_thread.native_handle(), "Executor thread");
  struct sched_param spin_thread_param;
  spin_thread_param.sched_priority = sched_get_priority_min(SCHED_FIFO);
  pthread_setschedparam(spin_thread.native_handle(), SCHED_FIFO, &spin_thread_param);
  spin_thread.detach();
  /*********************************************************************************************/

  /******** PUBLISHER THREAD ********************************************************************/
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
  /*********************************************************************************************/

  std::this_thread::sleep_for(30s);
  rclcpp::shutdown();
  print_stats("After shutdown");
}
