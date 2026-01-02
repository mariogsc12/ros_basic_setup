cmake_minimum_required(VERSION 3.8)
project(simple_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp)
find_package(std_msgs)

# Create executables
add_executable(simple_publisher src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher rclcpp std_msgs)

# Install include folder
include_directories(include)  # Important to found the include folder
install(
  DIRECTORY include
  DESTINATION include
)

# Install executables
install(TARGETS
  simple_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install python executables
install(PROGRAMS
  ${PROJECT_NAME}/simple_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
