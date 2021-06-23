cmake_minimum_required(VERSION 3.5)
project(benchmark_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num0.msg"
  "msg/Num1.msg"
  "msg/Num2.msg"
  "msg/Num3.msg"
  "msg/Num4.msg"
  "msg/Num5.msg"
  "msg/Num6.msg"
  "msg/Num7.msg"
  "msg/Num8.msg"
  "msg/Num9.msg"
  "msg/Num10.msg"
  "msg/Num11.msg"
  "msg/Num12.msg"
  "msg/Num13.msg"
  "msg/Num14.msg"
  "msg/Num15.msg"
  "msg/Num16.msg"
  "msg/Num17.msg"
  "msg/Num18.msg"
  "msg/Num19.msg"
  "msg/Num20.msg"
  "msg/Num21.msg"
  "msg/Num22.msg"
  "msg/Num23.msg"
  "msg/Num24.msg"
  "msg/Num25.msg"
  "msg/Num26.msg"
  "msg/Num27.msg"
  "msg/Num28.msg"
  "msg/Num29.msg"
  "msg/Num30.msg"
  "msg/Num31.msg"
  "msg/Num32.msg"
  "msg/Num33.msg"
  "msg/Num34.msg"
  "msg/Num35.msg"
  "msg/Num36.msg"
  "msg/Num37.msg"
  "msg/Num38.msg"
  "msg/Num39.msg"
  "msg/Num40.msg"
  "msg/Num41.msg"
  "msg/Num42.msg"
  "msg/Num43.msg"
  "msg/Num44.msg"
  "msg/Num45.msg"
  "msg/Num46.msg"
  "msg/Num47.msg"
  "msg/Num48.msg"
  "msg/Num49.msg"
  "msg/Num50.msg"
  "msg/Num51.msg"
  "msg/Num52.msg"
  "msg/Num53.msg"
  "msg/Num54.msg"
  "msg/Num55.msg"
  "msg/Num56.msg"
  "msg/Num57.msg"
  "msg/Num58.msg"
  "msg/Num59.msg"
  "msg/Num60.msg"
  "msg/Num61.msg"
  "msg/Num62.msg"
  "msg/Num63.msg"
  "msg/Num64.msg"
  "msg/Num65.msg"
  "msg/Num66.msg"
  "msg/Num67.msg"
  "msg/Num68.msg"
  "msg/Num69.msg"
  "msg/Num70.msg"
  "msg/Num71.msg"
  "msg/Num72.msg"
  "msg/Num73.msg"
  "msg/Num74.msg"
  "msg/Num75.msg"
  "msg/Num76.msg"
  "msg/Num77.msg"
  "msg/Num78.msg"
  "msg/Num79.msg"
  "msg/Num80.msg"
  "msg/Num81.msg"
  "msg/Num82.msg"
  "msg/Num83.msg"
  "msg/Num84.msg"
  "msg/Num85.msg"
  "msg/Num86.msg"
  "msg/Num87.msg"
  "msg/Num88.msg"
  "msg/Num89.msg"
  "msg/Num90.msg"
  "msg/Num91.msg"
  "msg/Num92.msg"
  "msg/Num93.msg"
  "msg/Num94.msg"
  "msg/Num95.msg"
  "msg/Num96.msg"
  "msg/Num97.msg"
  "msg/Num98.msg"
  "msg/Num99.msg"
 )

ament_package()
