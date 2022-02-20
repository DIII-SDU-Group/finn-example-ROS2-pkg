cmake_minimum_required(VERSION 3.5)
project(finn-example)

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

set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(include/finn-example-ip/ip/finn_fetcher/drivers/fetch_finn_v1_0/src)
link_directories (include/finn-example-ip/ip/finn_fetcher/drivers/fetch_finn_v1_0/src) 

include_directories(include/finn-example-ip/ip/stream_to_finn/drivers/streamToFinn_v1_0/src)
link_directories (include/finn-example-ip/ip/stream_to_finn/drivers/streamToFinn_v1_0/src)

add_executable(finn_interfacer 
    src/finn_interfacer_node.cpp
    include/finn-example-ip/ip/stream_to_finn/drivers/streamToFinn_v1_0/src/xstreamtofinn_linux.c
    include/finn-example-ip/ip/stream_to_finn/drivers/streamToFinn_v1_0/src/xstreamtofinn.c
    include/finn-example-ip/ip/finn_fetcher/drivers/fetch_finn_v1_0/src/xfetch_finn_linux.c
    include/finn-example-ip/ip/finn_fetcher/drivers/fetch_finn_v1_0/src/xfetch_finn.c
)

ament_target_dependencies(finn_interfacer 
    rclcpp 
    sensor_msgs
    std_msgs
    vision_msgs
    OpenCV
    cv_bridge
)

install(TARGETS
    finn_interfacer
    DESTINATION lib/${PROJECT_NAME}
)

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# install the config directory
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
