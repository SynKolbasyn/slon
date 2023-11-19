cmake_minimum_required(VERSION 3.0.2)
project(slon)

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_DEBUG_LIBS OFF)
set(Boost_USE_RELEASE_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  gps.msg
  motors.msg
  bt.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES slon
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_main_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_main_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME}_main_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_camera_node src/camera.cpp)
target_link_libraries(${PROJECT_NAME}_camera_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME}_camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_test_node test/main.cpp)
target_link_libraries(${PROJECT_NAME}_test_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME}_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
