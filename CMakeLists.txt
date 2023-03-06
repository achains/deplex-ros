cmake_minimum_required(VERSION 3.8)
project(deplex_ros)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(image_publisher src/image_publisher.cpp)
add_executable(image_listener src/image_listener.cpp)

ament_target_dependencies(image_publisher rclcpp std_msgs)
ament_target_dependencies(image_listener rclcpp std_msgs)

install(TARGETS
        image_publisher
        image_listener
        DESTINATION lib/${PROJECT_NAME})

ament_package()
