cmake_minimum_required(VERSION 3.8)
project(deplex_ros)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
include(FetchContent)

FetchContent_Declare(
        deplex
        GIT_REPOSITORY https://github.com/prime-slam/deplex.git
        GIT_TAG v1.0.5
)
FetchContent_GetProperties(deplex)
if (NOT deplex_POPULATED)
    FetchContent_Populate(deplex)
    set(BUILD_PYTHON OFF)
    set(BUILD_TESTS OFF)
    set(BUILD_EXAMPLES OFF)
    set(BUILD_SHARED_LIBS OFF)
    add_subdirectory(${deplex_SOURCE_DIR} ${deplex_BINARY_DIR})
endif()

#find_package(deplex REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(image_publisher src/image_publisher.cpp)
add_executable(image_listener src/image_listener.cpp)

ament_target_dependencies(image_publisher rclcpp std_msgs)
ament_target_dependencies(image_listener rclcpp std_msgs  )#deplex)

target_link_libraries(image_listener deplex)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
        image_publisher
        image_listener
        DESTINATION lib/${PROJECT_NAME})

ament_package()
