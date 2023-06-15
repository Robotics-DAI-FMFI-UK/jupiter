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
target_link_libraries(arm_test ${catkin_LIBRARIES})
rosbuild_add_executable(move_arm src/learning/move_arm.cpp)
rosbuild_add_executable(to_obstacle src/learning/to_obstacle.cpp)
rosbuild_add_executable(follow_me src/follow_me/follow_me.cpp)

