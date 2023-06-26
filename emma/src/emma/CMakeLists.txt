cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
set(CMAKE_CXX_STANDARD 11)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)

#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(talker src/learning/talker.cpp)
rosbuild_add_executable(listener src/learning/listener.cpp)

rosbuild_add_executable(add_two_ints_server src/learning/add_two_ints_server.cpp)
rosbuild_add_executable(add_two_ints_client src/learning/add_two_ints_client.cpp)



rosbuild_add_executable(arm_test src/learning/arm_to_xyz.cpp src/learning/jupiter_arm_kinematics.cpp)
rosbuild_add_executable(move_arm src/learning/move_arm.cpp)
rosbuild_add_executable(to_obstacle src/learning/to_obstacle.cpp)
rosbuild_add_executable(follow_me src/follow_me/follow_me.cpp)

find_package(OpenCV)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  image_transport
  OpenCV
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
rosbuild_add_executable(depth_map src/depth_map.cpp)
rosbuild_add_executable(save_depth src/save_depth.cpp)
rosbuild_add_executable(find_cube src/find_cube/find_cube.cpp src/find_cube/find_brick.cpp)
rosbuild_add_executable(catch_ball src/catch_ball/catch_ball.cpp src/catch_ball/find_brick.cpp src/catch_ball/arm_to_xyz.cpp src/catch_ball/camera_to_xyz.cpp src/catch_ball/jupiter_arm_kinematics.cpp src/catch_ball/camera_to_arm.cpp)

target_link_libraries(arm_test ${catkin_LIBRARIES})
target_link_libraries(depth_map ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(save_depth ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(find_cube ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(catch_ball ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBRARIES}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
