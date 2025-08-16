cmake_minimum_required(VERSION 3.5)
project(genz-icp-to-hdmapping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(rosbag2_cpp REQUIRED)

set(EXTERNAL_LIBRARIES_DIRECTORY src/3rdparty)

option(BUILD_WITH_BUNDLED_LIBLASZIP "Build with bundled Lib LASZIP" ON)
if (BUILD_WITH_BUNDLED_LIBLASZIP)
    add_subdirectory(${EXTERNAL_LIBRARIES_DIRECTORY}/LASzip)
    set(LASZIP_INCLUDE_DIR ${EXTERNAL_LIBRARIES_DIRECTORY})
    set(LASZIP_LIBRARY laszip)
    MESSAGE("Using bundled LASzip")
else()
    find_package(LASzip REQUIRED)
    set(LASZIP_INCLUDE_DIR ${LASZIP_INCLUDE_DIR})
    set(LASZIP_LIBRARY ${LASZIP_LIBRARY})
    set(PLATFORM_LASZIP_LIB ${LASZIP_LIBRARY})
    MESSAGE(STATUS "LASzip include dir: ${LASZIP_INCLUDE_DIR}", "LASzip library: ${LASZIP_LIBRARY}")
endif ()


          
add_executable(listener src/genz_odometry_subscriber.cpp src/laz_writer.cpp)

target_link_libraries(
  listener laszip
)

target_include_directories(
  listener
  PRIVATE ${LASZIP_INCLUDE_DIR}/LASzip/include)

ament_target_dependencies(listener rclcpp nav_msgs geometry_msgs sensor_msgs PCL rosbag2_cpp)



install(TARGETS
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
