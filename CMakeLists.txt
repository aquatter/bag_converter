cmake_minimum_required(VERSION 3.10.0)
project(bag_converter VERSION 0.1.0 LANGUAGES C CXX)

find_package(fmt REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED core imgproc imgcodecs videoio)
find_package(GeographicLib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(range-v3 REQUIRED)
find_package(nlohmann_json REQUIRED)

add_subdirectory(CLI11)

# get_property(importTargets DIRECTORY "${CMAKE_SOURCE_DIR}" PROPERTY IMPORTED_TARGETS)
# message("${importTargets}")

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    fmt::fmt
    rcl::rcl
    rosbag2_cpp::rosbag2_cpp
    rosbag2_transport::rosbag2_transport
    sensor_msgs::sensor_msgs_library
    CLI11
    ${GeographicLib_LIBRARIES}
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
)

add_executable(video_to_bag video_to_bag.cpp src/progress_bar.cpp)

target_include_directories(video_to_bag PRIVATE include)

target_link_libraries(video_to_bag
    ${OpenCV_LIBS}
    fmt::fmt
    rcl::rcl
    rosbag2_cpp::rosbag2_cpp
    rosbag2_transport::rosbag2_transport
    sensor_msgs::sensor_msgs_library
    CLI11
    Eigen3::Eigen
    ${GeographicLib_LIBRARIES}
    tinyxml2::tinyxml2
)

set_target_properties(video_to_bag PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
)


add_executable(gopro_to_bag
    gopro_to_bag.cpp
    src/progress_bar.cpp
    src/gpmf_parser.cpp
    src/gpmf_frame.cpp
    gpmf-parser/demo/GPMF_mp4reader.c
    gpmf-parser/demo/GPMF_print.c
    gpmf-parser/GPMF_parser.c
    gpmf-parser/GPMF_utils.c
)

target_include_directories(gopro_to_bag
    PRIVATE
    include
    gpmf-parser
    gpmf-parser/demo
)

target_link_libraries(gopro_to_bag
    ${OpenCV_LIBS}
    fmt::fmt
    rcl::rcl
    rosbag2_cpp::rosbag2_cpp
    rosbag2_transport::rosbag2_transport
    sensor_msgs::sensor_msgs_library
    CLI11
    Eigen3::Eigen
    ${GeographicLib_LIBRARIES}
    tinyxml2::tinyxml2
    range-v3::range-v3
    range-v3::meta
    range-v3::concepts
    nlohmann_json::nlohmann_json
    tinyxml2::tinyxml2
)

set_target_properties(gopro_to_bag PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
)

add_library(gpmf-extractor STATIC
    src/progress_bar.cpp
    src/gpmf_parser.cpp
    src/gpmf_frame.cpp
    gpmf-parser/demo/GPMF_mp4reader.c
    gpmf-parser/demo/GPMF_print.c
    gpmf-parser/GPMF_parser.c
    gpmf-parser/GPMF_utils.c
)

target_include_directories(gpmf-extractor PUBLIC
    include
    gpmf-parser
    gpmf-parser/demo
)

target_link_libraries(gpmf-extractor PUBLIC
    ${OpenCV_LIBS}
    fmt::fmt
    rcl::rcl
    rosbag2_cpp::rosbag2_cpp
    rosbag2_transport::rosbag2_transport
    sensor_msgs::sensor_msgs_library
    CLI11
    Eigen3::Eigen
    ${GeographicLib_LIBRARIES}
    tinyxml2::tinyxml2
    range-v3::range-v3
    range-v3::meta
    range-v3::concepts
    nlohmann_json::nlohmann_json
    tinyxml2::tinyxml2
    CLI11
)

set_target_properties(gpmf-extractor PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
)
