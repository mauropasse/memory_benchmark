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
#include "benchmark_interfaces/msg/num3.hpp"
#include "benchmark_interfaces/msg/num4.hpp"
#include "benchmark_interfaces/msg/num5.hpp"
#include "benchmark_interfaces/msg/num6.hpp"
#include "benchmark_interfaces/msg/num7.hpp"
#include "benchmark_interfaces/msg/num8.hpp"
#include "benchmark_interfaces/msg/num9.hpp"

#include "benchmark_interfaces/msg/num10.hpp"
#include "benchmark_interfaces/msg/num11.hpp"
#include "benchmark_interfaces/msg/num12.hpp"
#include "benchmark_interfaces/msg/num13.hpp"
#include "benchmark_interfaces/msg/num14.hpp"
#include "benchmark_interfaces/msg/num15.hpp"
#include "benchmark_interfaces/msg/num16.hpp"
#include "benchmark_interfaces/msg/num17.hpp"
#include "benchmark_interfaces/msg/num18.hpp"
#include "benchmark_interfaces/msg/num19.hpp"

#include "benchmark_interfaces/msg/num20.hpp"
#include "benchmark_interfaces/msg/num21.hpp"
#include "benchmark_interfaces/msg/num22.hpp"
#include "benchmark_interfaces/msg/num23.hpp"
#include "benchmark_interfaces/msg/num24.hpp"
#include "benchmark_interfaces/msg/num25.hpp"
#include "benchmark_interfaces/msg/num26.hpp"
#include "benchmark_interfaces/msg/num27.hpp"
#include "benchmark_interfaces/msg/num28.hpp"
#include "benchmark_interfaces/msg/num29.hpp"

#include "benchmark_interfaces/msg/num30.hpp"
#include "benchmark_interfaces/msg/num31.hpp"
#include "benchmark_interfaces/msg/num32.hpp"
#include "benchmark_interfaces/msg/num33.hpp"
#include "benchmark_interfaces/msg/num34.hpp"
#include "benchmark_interfaces/msg/num35.hpp"
#include "benchmark_interfaces/msg/num36.hpp"
#include "benchmark_interfaces/msg/num37.hpp"
#include "benchmark_interfaces/msg/num38.hpp"
#include "benchmark_interfaces/msg/num39.hpp"

#include "benchmark_interfaces/msg/num40.hpp"
#include "benchmark_interfaces/msg/num41.hpp"
#include "benchmark_interfaces/msg/num42.hpp"
#include "benchmark_interfaces/msg/num43.hpp"
#include "benchmark_interfaces/msg/num44.hpp"
#include "benchmark_interfaces/msg/num45.hpp"
#include "benchmark_interfaces/msg/num46.hpp"
#include "benchmark_interfaces/msg/num47.hpp"
#include "benchmark_interfaces/msg/num48.hpp"
#include "benchmark_interfaces/msg/num49.hpp"

#include "benchmark_interfaces/msg/num50.hpp"
#include "benchmark_interfaces/msg/num51.hpp"
#include "benchmark_interfaces/msg/num52.hpp"
#include "benchmark_interfaces/msg/num53.hpp"
#include "benchmark_interfaces/msg/num54.hpp"
#include "benchmark_interfaces/msg/num55.hpp"
#include "benchmark_interfaces/msg/num56.hpp"
#include "benchmark_interfaces/msg/num57.hpp"
#include "benchmark_interfaces/msg/num58.hpp"
#include "benchmark_interfaces/msg/num59.hpp"

#include "benchmark_interfaces/msg/num60.hpp"
#include "benchmark_interfaces/msg/num61.hpp"
#include "benchmark_interfaces/msg/num62.hpp"
#include "benchmark_interfaces/msg/num63.hpp"
#include "benchmark_interfaces/msg/num64.hpp"
#include "benchmark_interfaces/msg/num65.hpp"
#include "benchmark_interfaces/msg/num66.hpp"
#include "benchmark_interfaces/msg/num67.hpp"
#include "benchmark_interfaces/msg/num68.hpp"
#include "benchmark_interfaces/msg/num69.hpp"

#include "benchmark_interfaces/msg/num70.hpp"
#include "benchmark_interfaces/msg/num71.hpp"
#include "benchmark_interfaces/msg/num72.hpp"
#include "benchmark_interfaces/msg/num73.hpp"
#include "benchmark_interfaces/msg/num74.hpp"
#include "benchmark_interfaces/msg/num75.hpp"
#include "benchmark_interfaces/msg/num76.hpp"
#include "benchmark_interfaces/msg/num77.hpp"
#include "benchmark_interfaces/msg/num78.hpp"
#include "benchmark_interfaces/msg/num79.hpp"

#include "benchmark_interfaces/msg/num80.hpp"
#include "benchmark_interfaces/msg/num81.hpp"
#include "benchmark_interfaces/msg/num82.hpp"
#include "benchmark_interfaces/msg/num83.hpp"
#include "benchmark_interfaces/msg/num84.hpp"
#include "benchmark_interfaces/msg/num85.hpp"
#include "benchmark_interfaces/msg/num86.hpp"
#include "benchmark_interfaces/msg/num87.hpp"
#include "benchmark_interfaces/msg/num88.hpp"
#include "benchmark_interfaces/msg/num89.hpp"

#include "benchmark_interfaces/msg/num90.hpp"
#include "benchmark_interfaces/msg/num91.hpp"
#include "benchmark_interfaces/msg/num92.hpp"
#include "benchmark_interfaces/msg/num93.hpp"
#include "benchmark_interfaces/msg/num94.hpp"
#include "benchmark_interfaces/msg/num95.hpp"
#include "benchmark_interfaces/msg/num96.hpp"
#include "benchmark_interfaces/msg/num97.hpp"
#include "benchmark_interfaces/msg/num98.hpp"
#include "benchmark_interfaces/msg/num99.hpp"




using namespace std::chrono_literals;
using namespace std;

// using Executor = rclcpp::executors::StaticSingleThreadedExecutor;
// using Executor = rclcpp::executors::SingleThreadedExecutor;
using Executor = rclcpp::executors::EventsExecutor;

size_t count_{0};

auto start = std::chrono::high_resolution_clock::now();
auto finish = std::chrono::high_resolution_clock::now();

void print_stats(std::string str)
{
  static auto last_memory = 0;
  static size_t since_last_change = 0;
  struct rusage self_usage;
  getrusage(RUSAGE_SELF, &self_usage);
  // cout << str << ": " << self_usage.ru_maxrss << " Kb";
  auto diff_memory = self_usage.ru_maxrss - last_memory;
  // cout << " - Diff: " << diff_memory << " Kb" << endl;
  since_last_change++;


  if (diff_memory != 0) {
    cout << "MEMORY HAS CHANGED: " <<  diff_memory << " after changes:" << since_last_change << endl;
    since_last_change = 0;

  }
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
  subscription_ ## name = create_subscription< benchmark_interfaces::msg::Num ## msg_name>( \
    "topic_" #name, qos_depth, \
    [this](benchmark_interfaces::msg::Num ## msg_name::SharedPtr msg) { \
      RCLCPP_INFO(get_logger(), # name " got: %lld", msg->num); \
    }, sub_options); \
  publisher_ ## name = create_publisher< benchmark_interfaces::msg::Num ## msg_name>( \
    "topic_" #name, qos_depth, pub_options);

#define DECLARE_PUB_SUB(name, msg_name) \
  rclcpp::Subscription<benchmark_interfaces::msg::Num ## msg_name>::SharedPtr subscription_ ## name; \
  rclcpp::Publisher<benchmark_interfaces::msg::Num ## msg_name>::SharedPtr publisher_ ## name;

// #define CREATE_PUB_SUB(name, msg_name) \
//   auto subscription_ ## name = create_subscription< benchmark_interfaces::msg::Num0>( \
//     "topic_" #name, qos_depth, \
//     [this](benchmark_interfaces::msg::Num0::SharedPtr msg) { \
//     }, sub_options);
  // auto publisher_ ## name = create_publisher< benchmark_interfaces::msg::Num0>( \
  //   "topic_" #name, qos_depth, pub_options); \

// #define DECLARE_PUB_SUB(name, msg_name) \
//   rclcpp::Publisher<benchmark_interfaces::msg::Num0>::SharedPtr publisher_ ## name; \
//   rclcpp::Subscription<benchmark_interfaces::msg::Num0>::SharedPtr subscription_ ## name;

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

    // publisher = create_publisher<benchmark_interfaces::msg::Num0>(
    //   node_name, qos_depth, pub_options);

    // subscription = create_subscription<benchmark_interfaces::msg::Num0>(
    //   node_name, qos_depth,
    //   [this](benchmark_interfaces::msg::Num0::SharedPtr msg) {
    //     RCLCPP_INFO(get_logger(), " got: %lld", msg->num);
    //   },sub_options);

    CREATE_PUB_SUB(0,0) print_stats("");
    CREATE_PUB_SUB(1,1) print_stats("");
    CREATE_PUB_SUB(2,2) print_stats("");
    CREATE_PUB_SUB(3,3) print_stats("");
    CREATE_PUB_SUB(4,4) print_stats("");
    CREATE_PUB_SUB(5,5) print_stats("");
    CREATE_PUB_SUB(6,6) print_stats("");
    CREATE_PUB_SUB(7,7) print_stats("");
    CREATE_PUB_SUB(8,8) print_stats("");
    CREATE_PUB_SUB(9,9) print_stats("");
    CREATE_PUB_SUB(10,10) print_stats("");
    CREATE_PUB_SUB(11,11) print_stats("");
    CREATE_PUB_SUB(12,12) print_stats("");
    CREATE_PUB_SUB(13,13) print_stats("");
    CREATE_PUB_SUB(14,14) print_stats("");
    CREATE_PUB_SUB(15,15) print_stats("");
    CREATE_PUB_SUB(16,16) print_stats("");
    CREATE_PUB_SUB(17,17) print_stats("");
    CREATE_PUB_SUB(18,18) print_stats("");
    CREATE_PUB_SUB(19,19) print_stats("");
    CREATE_PUB_SUB(20,20) print_stats("");
    CREATE_PUB_SUB(21,21) print_stats("");
    CREATE_PUB_SUB(22,22) print_stats("");
    CREATE_PUB_SUB(23,23) print_stats("");
    CREATE_PUB_SUB(24,24) print_stats("");
    CREATE_PUB_SUB(25,25) print_stats("");
    CREATE_PUB_SUB(26,26) print_stats("");
    CREATE_PUB_SUB(27,27) print_stats("");
    CREATE_PUB_SUB(28,28) print_stats("");
    CREATE_PUB_SUB(29,29) print_stats("");
    CREATE_PUB_SUB(30,30) print_stats("");
    CREATE_PUB_SUB(31,31) print_stats("");
    CREATE_PUB_SUB(32,32) print_stats("");
    CREATE_PUB_SUB(33,33) print_stats("");
    CREATE_PUB_SUB(34,34) print_stats("");
    CREATE_PUB_SUB(35,35) print_stats("");
    CREATE_PUB_SUB(36,36) print_stats("");
    CREATE_PUB_SUB(37,37) print_stats("");
    CREATE_PUB_SUB(38,38) print_stats("");
    CREATE_PUB_SUB(39,39) print_stats("");
    CREATE_PUB_SUB(40,40) print_stats("");
    CREATE_PUB_SUB(41,41) print_stats("");
    CREATE_PUB_SUB(42,42) print_stats("");
    CREATE_PUB_SUB(43,43) print_stats("");
    CREATE_PUB_SUB(44,44) print_stats("");
    CREATE_PUB_SUB(45,45) print_stats("");
    CREATE_PUB_SUB(46,46) print_stats("");
    CREATE_PUB_SUB(47,47) print_stats("");
    CREATE_PUB_SUB(48,48) print_stats("");
    CREATE_PUB_SUB(49,49) print_stats("");
    CREATE_PUB_SUB(50,50) print_stats("");
    CREATE_PUB_SUB(51,51) print_stats("");
    CREATE_PUB_SUB(52,52) print_stats("");
    CREATE_PUB_SUB(53,53) print_stats("");
    CREATE_PUB_SUB(54,54) print_stats("");
    CREATE_PUB_SUB(55,55) print_stats("");
    CREATE_PUB_SUB(56,56) print_stats("");
    CREATE_PUB_SUB(57,57) print_stats("");
    CREATE_PUB_SUB(58,58) print_stats("");
    CREATE_PUB_SUB(59,59) print_stats("");
    CREATE_PUB_SUB(60,60) print_stats("");
    CREATE_PUB_SUB(61,61) print_stats("");
    CREATE_PUB_SUB(62,62) print_stats("");
    CREATE_PUB_SUB(63,63) print_stats("");
    CREATE_PUB_SUB(64,64) print_stats("");
    CREATE_PUB_SUB(65,65) print_stats("");
    CREATE_PUB_SUB(66,66) print_stats("");
    CREATE_PUB_SUB(67,67) print_stats("");
    CREATE_PUB_SUB(68,68) print_stats("");
    CREATE_PUB_SUB(69,69) print_stats("");
    CREATE_PUB_SUB(70,70) print_stats("");
    CREATE_PUB_SUB(71,71) print_stats("");
    CREATE_PUB_SUB(72,72) print_stats("");
    CREATE_PUB_SUB(73,73) print_stats("");
    CREATE_PUB_SUB(74,74) print_stats("");
    CREATE_PUB_SUB(75,75) print_stats("");
    CREATE_PUB_SUB(76,76) print_stats("");
    CREATE_PUB_SUB(77,77) print_stats("");
    CREATE_PUB_SUB(78,78) print_stats("");
    CREATE_PUB_SUB(79,79) print_stats("");
    CREATE_PUB_SUB(80,80) print_stats("");
    CREATE_PUB_SUB(81,81) print_stats("");
    CREATE_PUB_SUB(82,82) print_stats("");
    CREATE_PUB_SUB(83,83) print_stats("");
    CREATE_PUB_SUB(84,84) print_stats("");
    CREATE_PUB_SUB(85,85) print_stats("");
    CREATE_PUB_SUB(86,86) print_stats("");
    CREATE_PUB_SUB(87,87) print_stats("");
    CREATE_PUB_SUB(88,88) print_stats("");
    CREATE_PUB_SUB(89,89) print_stats("");
    CREATE_PUB_SUB(90,90) print_stats("");
    CREATE_PUB_SUB(91,91) print_stats("");
    CREATE_PUB_SUB(92,92) print_stats("");
    CREATE_PUB_SUB(93,93) print_stats("");
    CREATE_PUB_SUB(94,94) print_stats("");
    CREATE_PUB_SUB(95,95) print_stats("");
    CREATE_PUB_SUB(96,96) print_stats("");
    CREATE_PUB_SUB(97,97) print_stats("");
    CREATE_PUB_SUB(98,98) print_stats("");
    CREATE_PUB_SUB(99,99) print_stats("");

    // CREATE_PUB_SUB(100,0) print_stats("After create publishers");
    // CREATE_PUB_SUB(101,1) print_stats("After create publishers");
    // CREATE_PUB_SUB(102,2) print_stats("After create publishers");
    // CREATE_PUB_SUB(103,3) print_stats("After create publishers");
    // CREATE_PUB_SUB(104,4) print_stats("After create publishers");
    // CREATE_PUB_SUB(105,5) print_stats("After create publishers");
    // CREATE_PUB_SUB(106,6) print_stats("After create publishers");
    // CREATE_PUB_SUB(107,7) print_stats("After create publishers");
    // CREATE_PUB_SUB(108,8) print_stats("After create publishers");
    // CREATE_PUB_SUB(109,9) print_stats("After create publishers");
    // CREATE_PUB_SUB(110,10) print_stats("After create publishers");
    // CREATE_PUB_SUB(111,11) print_stats("After create publishers");
    // CREATE_PUB_SUB(112,12) print_stats("After create publishers");
    // CREATE_PUB_SUB(113,13) print_stats("After create publishers");
    // CREATE_PUB_SUB(114,14) print_stats("After create publishers");
    // CREATE_PUB_SUB(115,15) print_stats("After create publishers");
    // CREATE_PUB_SUB(116,16) print_stats("After create publishers");
    // CREATE_PUB_SUB(117,17) print_stats("After create publishers");
    // CREATE_PUB_SUB(118,18) print_stats("After create publishers");
    // CREATE_PUB_SUB(119,19) print_stats("After create publishers");
    // CREATE_PUB_SUB(120,20) print_stats("After create publishers");
    // CREATE_PUB_SUB(121,21) print_stats("After create publishers");
    // CREATE_PUB_SUB(122,22) print_stats("After create publishers");
    // CREATE_PUB_SUB(123,23) print_stats("After create publishers");
    // CREATE_PUB_SUB(124,24) print_stats("After create publishers");
    // CREATE_PUB_SUB(125,25) print_stats("After create publishers");
    // CREATE_PUB_SUB(126,26) print_stats("After create publishers");
    // CREATE_PUB_SUB(127,27) print_stats("After create publishers");
    // CREATE_PUB_SUB(128,28) print_stats("After create publishers");
    // CREATE_PUB_SUB(129,29) print_stats("After create publishers");
    // CREATE_PUB_SUB(130,30) print_stats("After create publishers");
    // CREATE_PUB_SUB(131,31) print_stats("After create publishers");
    // CREATE_PUB_SUB(132,32) print_stats("After create publishers");
    // CREATE_PUB_SUB(133,33) print_stats("After create publishers");
    // CREATE_PUB_SUB(134,34) print_stats("After create publishers");
    // CREATE_PUB_SUB(135,35) print_stats("After create publishers");
    // CREATE_PUB_SUB(136,36) print_stats("After create publishers");
    // CREATE_PUB_SUB(137,37) print_stats("After create publishers");
    // CREATE_PUB_SUB(138,38) print_stats("After create publishers");
    // CREATE_PUB_SUB(139,39) print_stats("After create publishers");
    // CREATE_PUB_SUB(140,40) print_stats("After create publishers");
    // CREATE_PUB_SUB(141,41) print_stats("After create publishers");
    // CREATE_PUB_SUB(142,42) print_stats("After create publishers");
    // CREATE_PUB_SUB(143,43) print_stats("After create publishers");
    // CREATE_PUB_SUB(144,44) print_stats("After create publishers");
    // CREATE_PUB_SUB(145,45) print_stats("After create publishers");
    // CREATE_PUB_SUB(146,46) print_stats("After create publishers");
    // CREATE_PUB_SUB(147,47) print_stats("After create publishers");
    // CREATE_PUB_SUB(148,48) print_stats("After create publishers");
    // CREATE_PUB_SUB(149,49) print_stats("After create publishers");
    // CREATE_PUB_SUB(150,40) print_stats("After create publishers");
    // CREATE_PUB_SUB(151,41) print_stats("After create publishers");
    // CREATE_PUB_SUB(152,42) print_stats("After create publishers");
    // CREATE_PUB_SUB(153,43) print_stats("After create publishers");
    // CREATE_PUB_SUB(154,44) print_stats("After create publishers");
    // CREATE_PUB_SUB(155,45) print_stats("After create publishers");
    // CREATE_PUB_SUB(156,46) print_stats("After create publishers");
    // CREATE_PUB_SUB(157,47) print_stats("After create publishers");
    // CREATE_PUB_SUB(158,48) print_stats("After create publishers");
    // CREATE_PUB_SUB(159,49) print_stats("After create publishers");
    // CREATE_PUB_SUB(160,40) print_stats("After create publishers");
    // CREATE_PUB_SUB(161,41) print_stats("After create publishers");
    // CREATE_PUB_SUB(162,42) print_stats("After create publishers");
    // CREATE_PUB_SUB(163,43) print_stats("After create publishers");
    // CREATE_PUB_SUB(164,44) print_stats("After create publishers");
    // CREATE_PUB_SUB(165,45) print_stats("After create publishers");
    // CREATE_PUB_SUB(166,46) print_stats("After create publishers");
    // CREATE_PUB_SUB(167,47) print_stats("After create publishers");
    // CREATE_PUB_SUB(168,48) print_stats("After create publishers");
    // CREATE_PUB_SUB(169,49) print_stats("After create publishers");
    // CREATE_PUB_SUB(170,40) print_stats("After create publishers");
    // CREATE_PUB_SUB(171,41) print_stats("After create publishers");
    // CREATE_PUB_SUB(172,42) print_stats("After create publishers");
    // CREATE_PUB_SUB(173,43) print_stats("After create publishers");
    // CREATE_PUB_SUB(174,44) print_stats("After create publishers");
    // CREATE_PUB_SUB(175,45) print_stats("After create publishers");
    // CREATE_PUB_SUB(176,46) print_stats("After create publishers");
    // CREATE_PUB_SUB(177,47) print_stats("After create publishers");
    // CREATE_PUB_SUB(178,48) print_stats("After create publishers");
    // CREATE_PUB_SUB(179,49) print_stats("After create publishers");
    // CREATE_PUB_SUB(180,40) print_stats("After create publishers");
    // CREATE_PUB_SUB(181,41) print_stats("After create publishers");
    // CREATE_PUB_SUB(182,42) print_stats("After create publishers");
    // CREATE_PUB_SUB(183,43) print_stats("After create publishers");
    // CREATE_PUB_SUB(184,44) print_stats("After create publishers");
    // CREATE_PUB_SUB(185,45) print_stats("After create publishers");
    // CREATE_PUB_SUB(186,46) print_stats("After create publishers");
    // CREATE_PUB_SUB(187,47) print_stats("After create publishers");
    // CREATE_PUB_SUB(188,48) print_stats("After create publishers");
    // CREATE_PUB_SUB(189,49) print_stats("After create publishers");
    // CREATE_PUB_SUB(190,40) print_stats("After create publishers");
    // CREATE_PUB_SUB(191,41) print_stats("After create publishers");
    // CREATE_PUB_SUB(192,42) print_stats("After create publishers");
    // CREATE_PUB_SUB(193,43) print_stats("After create publishers");
    // CREATE_PUB_SUB(194,44) print_stats("After create publishers");
    // CREATE_PUB_SUB(195,45) print_stats("After create publishers");
    // CREATE_PUB_SUB(196,46) print_stats("After create publishers");
    // CREATE_PUB_SUB(197,47) print_stats("After create publishers");
    // CREATE_PUB_SUB(198,48) print_stats("After create publishers");
    // CREATE_PUB_SUB(199,49) print_stats("After create publishers");

    // print_stats("After create subscribers");
  }

#define PUBLISH(name, msg_name) \
  auto message ## name = std::make_unique<benchmark_interfaces::msg::Num ## msg_name>(); \
  message ## name->num = count_; \
  publisher_ ## name->publish(std::move(message ## name));

  void publish_messages()
  {
    // auto message = std::make_unique<benchmark_interfaces::msg::Num0>();
    // message->num = count_;
    // publisher->publish(std::move(message));
    PUBLISH(0,0)
    PUBLISH(1,1)
    PUBLISH(2,2)
    PUBLISH(3,3)
    PUBLISH(4,4)
    PUBLISH(5,5)
    PUBLISH(6,6)
    PUBLISH(7,7)
    PUBLISH(8,8)
    PUBLISH(9,9)

    PUBLISH(10,10)
    PUBLISH(11,11)
    PUBLISH(12,12)
    PUBLISH(13,13)
    PUBLISH(14,14)
    PUBLISH(15,15)
    PUBLISH(16,16)
    PUBLISH(17,17)
    PUBLISH(18,18)
    PUBLISH(19,19)

    PUBLISH(20,20)
    PUBLISH(21,21)
    PUBLISH(22,22)
    PUBLISH(23,23)
    PUBLISH(24,24)
    PUBLISH(25,25)
    PUBLISH(26,26)
    PUBLISH(27,27)
    PUBLISH(28,28)
    PUBLISH(29,29)

    PUBLISH(30,30)
    PUBLISH(31,31)
    PUBLISH(32,32)
    PUBLISH(33,33)
    PUBLISH(34,34)
    PUBLISH(35,35)
    PUBLISH(36,36)
    PUBLISH(37,37)
    PUBLISH(38,38)
    PUBLISH(39,39)

    PUBLISH(40,40)
    PUBLISH(41,41)
    PUBLISH(42,42)
    PUBLISH(43,43)
    PUBLISH(44,44)
    PUBLISH(45,45)
    PUBLISH(46,46)
    PUBLISH(47,47)
    PUBLISH(48,48)
    PUBLISH(49,49)

    PUBLISH(50,50)
    PUBLISH(51,51)
    PUBLISH(52,52)
    PUBLISH(53,53)
    PUBLISH(54,54)
    PUBLISH(55,55)
    PUBLISH(56,56)
    PUBLISH(57,57)
    PUBLISH(58,58)
    PUBLISH(59,59)
    PUBLISH(60,60)
    PUBLISH(61,61)
    PUBLISH(62,62)
    PUBLISH(63,63)
    PUBLISH(64,64)
    PUBLISH(65,65)
    PUBLISH(66,66)
    PUBLISH(67,67)
    PUBLISH(68,68)
    PUBLISH(69,69)
    PUBLISH(70,70)
    PUBLISH(71,71)
    PUBLISH(72,72)
    PUBLISH(73,73)
    PUBLISH(74,74)
    PUBLISH(75,75)
    PUBLISH(76,76)
    PUBLISH(77,77)
    PUBLISH(78,78)
    PUBLISH(79,79)
    PUBLISH(80,80)
    PUBLISH(81,81)
    PUBLISH(82,82)
    PUBLISH(83,83)
    PUBLISH(84,84)
    PUBLISH(85,85)
    PUBLISH(86,86)
    PUBLISH(87,87)
    PUBLISH(88,88)
    PUBLISH(89,89)
    PUBLISH(90,90)
    PUBLISH(91,91)
    PUBLISH(92,92)
    PUBLISH(93,93)
    PUBLISH(94,94)
    PUBLISH(95,95)
    PUBLISH(96,96)
    PUBLISH(97,97)
    PUBLISH(98,98)
    PUBLISH(99,99)
  }

private:
  // rclcpp::Subscription<benchmark_interfaces::msg::Num0>::SharedPtr subscription;
  // rclcpp::Publisher<benchmark_interfaces::msg::Num0>::SharedPtr publisher;

  DECLARE_PUB_SUB(0,0)
  DECLARE_PUB_SUB(1,1)
  DECLARE_PUB_SUB(2,2)
  DECLARE_PUB_SUB(3,3)
  DECLARE_PUB_SUB(4,4)
  DECLARE_PUB_SUB(5,5)
  DECLARE_PUB_SUB(6,6)
  DECLARE_PUB_SUB(7,7)
  DECLARE_PUB_SUB(8,8)
  DECLARE_PUB_SUB(9,9)
  DECLARE_PUB_SUB(10,10)
  DECLARE_PUB_SUB(11,11)
  DECLARE_PUB_SUB(12,12)
  DECLARE_PUB_SUB(13,13)
  DECLARE_PUB_SUB(14,14)
  DECLARE_PUB_SUB(15,15)
  DECLARE_PUB_SUB(16,16)
  DECLARE_PUB_SUB(17,17)
  DECLARE_PUB_SUB(18,18)
  DECLARE_PUB_SUB(19,19)
  DECLARE_PUB_SUB(20,20)
  DECLARE_PUB_SUB(21,21)
  DECLARE_PUB_SUB(22,22)
  DECLARE_PUB_SUB(23,23)
  DECLARE_PUB_SUB(24,24)
  DECLARE_PUB_SUB(25,25)
  DECLARE_PUB_SUB(26,26)
  DECLARE_PUB_SUB(27,27)
  DECLARE_PUB_SUB(28,28)
  DECLARE_PUB_SUB(29,29)
  DECLARE_PUB_SUB(30,30)
  DECLARE_PUB_SUB(31,31)
  DECLARE_PUB_SUB(32,32)
  DECLARE_PUB_SUB(33,33)
  DECLARE_PUB_SUB(34,34)
  DECLARE_PUB_SUB(35,35)
  DECLARE_PUB_SUB(36,36)
  DECLARE_PUB_SUB(37,37)
  DECLARE_PUB_SUB(38,38)
  DECLARE_PUB_SUB(39,39)
  DECLARE_PUB_SUB(40,40)
  DECLARE_PUB_SUB(41,41)
  DECLARE_PUB_SUB(42,42)
  DECLARE_PUB_SUB(43,43)
  DECLARE_PUB_SUB(44,44)
  DECLARE_PUB_SUB(45,45)
  DECLARE_PUB_SUB(46,46)
  DECLARE_PUB_SUB(47,47)
  DECLARE_PUB_SUB(48,48)
  DECLARE_PUB_SUB(49,49)
  DECLARE_PUB_SUB(50,50)
  DECLARE_PUB_SUB(51,51)
  DECLARE_PUB_SUB(52,52)
  DECLARE_PUB_SUB(53,53)
  DECLARE_PUB_SUB(54,54)
  DECLARE_PUB_SUB(55,55)
  DECLARE_PUB_SUB(56,56)
  DECLARE_PUB_SUB(57,57)
  DECLARE_PUB_SUB(58,58)
  DECLARE_PUB_SUB(59,59)
  DECLARE_PUB_SUB(60,60)
  DECLARE_PUB_SUB(61,61)
  DECLARE_PUB_SUB(62,62)
  DECLARE_PUB_SUB(63,63)
  DECLARE_PUB_SUB(64,64)
  DECLARE_PUB_SUB(65,65)
  DECLARE_PUB_SUB(66,66)
  DECLARE_PUB_SUB(67,67)
  DECLARE_PUB_SUB(68,68)
  DECLARE_PUB_SUB(69,69)
  DECLARE_PUB_SUB(70,70)
  DECLARE_PUB_SUB(71,71)
  DECLARE_PUB_SUB(72,72)
  DECLARE_PUB_SUB(73,73)
  DECLARE_PUB_SUB(74,74)
  DECLARE_PUB_SUB(75,75)
  DECLARE_PUB_SUB(76,76)
  DECLARE_PUB_SUB(77,77)
  DECLARE_PUB_SUB(78,78)
  DECLARE_PUB_SUB(79,79)
  DECLARE_PUB_SUB(80,80)
  DECLARE_PUB_SUB(81,81)
  DECLARE_PUB_SUB(82,82)
  DECLARE_PUB_SUB(83,83)
  DECLARE_PUB_SUB(84,84)
  DECLARE_PUB_SUB(85,85)
  DECLARE_PUB_SUB(86,86)
  DECLARE_PUB_SUB(87,87)
  DECLARE_PUB_SUB(88,88)
  DECLARE_PUB_SUB(89,89)
  DECLARE_PUB_SUB(90,90)
  DECLARE_PUB_SUB(91,91)
  DECLARE_PUB_SUB(92,92)
  DECLARE_PUB_SUB(93,93)
  DECLARE_PUB_SUB(94,94)
  DECLARE_PUB_SUB(95,95)
  DECLARE_PUB_SUB(96,96)
  DECLARE_PUB_SUB(97,97)
  DECLARE_PUB_SUB(98,98)
  DECLARE_PUB_SUB(99,99)
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
  std::this_thread::sleep_for(2s);
  /*********************************************************************************************/

  /******** EXECUTOR THREAD ********************************************************************/
  auto executor = std::make_shared<Executor>();
  executor->add_node(my_node0);

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
        my_node0->publish_messages();
        std::this_thread::sleep_for(5s);
        count_++; cout << endl;
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


















  // executor->add_node(my_node1);
  // executor->add_node(my_node2);
  // executor->add_node(my_node3);
  // executor->add_node(my_node4);
  // executor->add_node(my_node5);
  // executor->add_node(my_node6);
  // executor->add_node(my_node7);
  // executor->add_node(my_node8);
  // executor->add_node(my_node9);

  // executor->add_node(my_node10);
  // executor->add_node(my_node11);
  // executor->add_node(my_node12);
  // executor->add_node(my_node13);
  // executor->add_node(my_node14);
  // executor->add_node(my_node15);
  // executor->add_node(my_node16);
  // executor->add_node(my_node17);
  // executor->add_node(my_node18);
  // executor->add_node(my_node19);

  // executor->add_node(my_node20);
  // executor->add_node(my_node21);
  // executor->add_node(my_node22);
  // executor->add_node(my_node23);
  // executor->add_node(my_node24);
  // executor->add_node(my_node25);
  // executor->add_node(my_node26);
  // executor->add_node(my_node27);
  // executor->add_node(my_node28);
  // executor->add_node(my_node29);

  // executor->add_node(my_node30);
  // executor->add_node(my_node31);
  // executor->add_node(my_node32);
  // executor->add_node(my_node33);
  // executor->add_node(my_node34);
  // executor->add_node(my_node35);
  // executor->add_node(my_node36);
  // executor->add_node(my_node37);
  // executor->add_node(my_node38);
  // executor->add_node(my_node39);

  // executor->add_node(my_node40);
  // executor->add_node(my_node41);
  // executor->add_node(my_node42);
  // executor->add_node(my_node43);
  // executor->add_node(my_node44);
  // executor->add_node(my_node45);
  // executor->add_node(my_node46);
  // executor->add_node(my_node47);
  // executor->add_node(my_node48);
  // executor->add_node(my_node49);

        // my_node1->publish_messages();
        // my_node2->publish_messages();
        // my_node3->publish_messages();
        // my_node4->publish_messages();
        // my_node5->publish_messages();
        // my_node6->publish_messages();
        // my_node7->publish_messages();
        // my_node8->publish_messages();
        // my_node9->publish_messages();

        // my_node10->publish_messages();
        // my_node11->publish_messages();
        // my_node12->publish_messages();
        // my_node13->publish_messages();
        // my_node14->publish_messages();
        // my_node15->publish_messages();
        // my_node16->publish_messages();
        // my_node17->publish_messages();
        // my_node18->publish_messages();
        // my_node19->publish_messages();

        // my_node20->publish_messages();
        // my_node21->publish_messages();
        // my_node22->publish_messages();
        // my_node23->publish_messages();
        // my_node24->publish_messages();
        // my_node25->publish_messages();
        // my_node26->publish_messages();
        // my_node27->publish_messages();
        // my_node28->publish_messages();
        // my_node29->publish_messages();

        // my_node30->publish_messages();
        // my_node31->publish_messages();
        // my_node32->publish_messages();
        // my_node33->publish_messages();
        // my_node34->publish_messages();
        // my_node35->publish_messages();
        // my_node36->publish_messages();
        // my_node37->publish_messages();
        // my_node38->publish_messages();
        // my_node39->publish_messages();

        // my_node40->publish_messages();
        // my_node41->publish_messages();
        // my_node42->publish_messages();
        // my_node43->publish_messages();
        // my_node44->publish_messages();
        // my_node45->publish_messages();
        // my_node46->publish_messages();
        // my_node47->publish_messages();
        // my_node48->publish_messages();
        // my_node49->publish_messages();

  // auto my_node1 = std::make_shared<MyNode>("my_node1", node_options);
  // auto my_node2 = std::make_shared<MyNode>("my_node2", node_options);
  // auto my_node3 = std::make_shared<MyNode>("my_node3", node_options);
  // auto my_node4 = std::make_shared<MyNode>("my_node4", node_options);
  // auto my_node5 = std::make_shared<MyNode>("my_node5", node_options);
  // auto my_node6 = std::make_shared<MyNode>("my_node6", node_options);
  // auto my_node7 = std::make_shared<MyNode>("my_node7", node_options);
  // auto my_node8 = std::make_shared<MyNode>("my_node8", node_options);
  // auto my_node9 = std::make_shared<MyNode>("my_node9", node_options);

  // auto my_node10 = std::make_shared<MyNode>("my_node10", node_options);
  // auto my_node11 = std::make_shared<MyNode>("my_node11", node_options);
  // auto my_node12 = std::make_shared<MyNode>("my_node12", node_options);
  // auto my_node13 = std::make_shared<MyNode>("my_node13", node_options);
  // auto my_node14 = std::make_shared<MyNode>("my_node14", node_options);
  // auto my_node15 = std::make_shared<MyNode>("my_node15", node_options);
  // auto my_node16 = std::make_shared<MyNode>("my_node16", node_options);
  // auto my_node17 = std::make_shared<MyNode>("my_node17", node_options);
  // auto my_node18 = std::make_shared<MyNode>("my_node18", node_options);
  // auto my_node19 = std::make_shared<MyNode>("my_node19", node_options);

  // auto my_node20 = std::make_shared<MyNode>("my_node20", node_options);
  // auto my_node21 = std::make_shared<MyNode>("my_node21", node_options);
  // auto my_node22 = std::make_shared<MyNode>("my_node22", node_options);
  // auto my_node23 = std::make_shared<MyNode>("my_node23", node_options);
  // auto my_node24 = std::make_shared<MyNode>("my_node24", node_options);
  // auto my_node25 = std::make_shared<MyNode>("my_node25", node_options);
  // auto my_node26 = std::make_shared<MyNode>("my_node26", node_options);
  // auto my_node27 = std::make_shared<MyNode>("my_node27", node_options);
  // auto my_node28 = std::make_shared<MyNode>("my_node28", node_options);
  // auto my_node29 = std::make_shared<MyNode>("my_node29", node_options);

  // auto my_node30 = std::make_shared<MyNode>("my_node30", node_options);
  // auto my_node31 = std::make_shared<MyNode>("my_node31", node_options);
  // auto my_node32 = std::make_shared<MyNode>("my_node32", node_options);
  // auto my_node33 = std::make_shared<MyNode>("my_node33", node_options);
  // auto my_node34 = std::make_shared<MyNode>("my_node34", node_options);
  // auto my_node35 = std::make_shared<MyNode>("my_node35", node_options);
  // auto my_node36 = std::make_shared<MyNode>("my_node36", node_options);
  // auto my_node37 = std::make_shared<MyNode>("my_node37", node_options);
  // auto my_node38 = std::make_shared<MyNode>("my_node38", node_options);
  // auto my_node39 = std::make_shared<MyNode>("my_node39", node_options);

  // auto my_node40 = std::make_shared<MyNode>("my_node40", node_options);
  // auto my_node41 = std::make_shared<MyNode>("my_node41", node_options);
  // auto my_node42 = std::make_shared<MyNode>("my_node42", node_options);
  // auto my_node43 = std::make_shared<MyNode>("my_node43", node_options);
  // auto my_node44 = std::make_shared<MyNode>("my_node44", node_options);
  // auto my_node45 = std::make_shared<MyNode>("my_node45", node_options);
  // auto my_node46 = std::make_shared<MyNode>("my_node46", node_options);
  // auto my_node47 = std::make_shared<MyNode>("my_node47", node_options);
  // auto my_node48 = std::make_shared<MyNode>("my_node48", node_options);
  // auto my_node49 = std::make_shared<MyNode>("my_node49", node_options);

  // DECLARE_PUB_SUB(0,0)
  // DECLARE_PUB_SUB(1,1)
  // DECLARE_PUB_SUB(2,2)
  // DECLARE_PUB_SUB(3,3)
  // DECLARE_PUB_SUB(4,4)
  // DECLARE_PUB_SUB(5,5)
  // DECLARE_PUB_SUB(6,6)
  // DECLARE_PUB_SUB(7,7)
  // DECLARE_PUB_SUB(8,8)
  // DECLARE_PUB_SUB(9,9)

  // DECLARE_PUB_SUB(10,10)
  // DECLARE_PUB_SUB(11,11)
  // DECLARE_PUB_SUB(12,12)
  // DECLARE_PUB_SUB(13,13)
  // DECLARE_PUB_SUB(14,14)
  // DECLARE_PUB_SUB(15,15)
  // DECLARE_PUB_SUB(16,16)
  // DECLARE_PUB_SUB(17,17)
  // DECLARE_PUB_SUB(18,18)
  // DECLARE_PUB_SUB(19,19)

  // DECLARE_PUB_SUB(20,20)
  // DECLARE_PUB_SUB(21,21)
  // DECLARE_PUB_SUB(22,22)
  // DECLARE_PUB_SUB(23,23)
  // DECLARE_PUB_SUB(24,24)
  // DECLARE_PUB_SUB(25,25)
  // DECLARE_PUB_SUB(26,26)
  // DECLARE_PUB_SUB(27,27)
  // DECLARE_PUB_SUB(28,28)
  // DECLARE_PUB_SUB(29,29)

  // DECLARE_PUB_SUB(30,30)
  // DECLARE_PUB_SUB(31,31)
  // DECLARE_PUB_SUB(32,32)
  // DECLARE_PUB_SUB(33,33)
  // DECLARE_PUB_SUB(34,34)
  // DECLARE_PUB_SUB(35,35)
  // DECLARE_PUB_SUB(36,36)
  // DECLARE_PUB_SUB(37,37)
  // DECLARE_PUB_SUB(38,38)
  // DECLARE_PUB_SUB(39,39)

  // DECLARE_PUB_SUB(40,40)
  // DECLARE_PUB_SUB(41,41)
  // DECLARE_PUB_SUB(42,42)
  // DECLARE_PUB_SUB(43,43)
  // DECLARE_PUB_SUB(44,44)
  // DECLARE_PUB_SUB(45,45)
  // DECLARE_PUB_SUB(46,46)
  // DECLARE_PUB_SUB(47,47)
  // DECLARE_PUB_SUB(48,48)
  // DECLARE_PUB_SUB(49,49)

