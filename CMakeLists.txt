cmake_minimum_required(VERSION 3.20)
project(summit_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robotnik_description REQUIRED)
find_package(robotnik_sensors REQUIRED)

install(DIRECTORY launch rviz worlds
        DESTINATION share/${PROJECT_NAME})

ament_package()