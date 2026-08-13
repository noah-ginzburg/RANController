cmake_minimum_required(VERSION 3.5)
project(vicon_receiver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Position.msg"
  "msg/PositionList.msg"
    DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(vicon_client src/vicon_receiver/communicator.cpp src/vicon_receiver/publisher.cpp)
target_link_libraries(vicon_client ViconDataStreamSDK_CPP ${Boost_LIBRARIES})
ament_target_dependencies(vicon_client "rclcpp")

get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)
get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

foreach(typesupport_impl ${typesupport_impls})
  rosidl_target_interfaces(vicon_client
    ${PROJECT_NAME} "rosidl_typesupport_cpp"
  )
endforeach()

install(TARGETS vicon_client DESTINATION lib/${PROJECT_NAME})

# vicon_bridge.py is a standalone rclpy node (not a source file of vicon_client --
# it was previously listed as one of the add_executable() sources above, which
# doesn't work: a .py file can't compile into a C++ target, and this cmake-only
# package never had an install rule that puts it in lib/vicon_receiver/, which is
# why `ros2 launch`/`ros2 run` couldn't find it as an executable).
install(PROGRAMS
  src/vicon_receiver/vicon_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch file
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
