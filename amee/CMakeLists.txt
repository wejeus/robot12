cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include_directories(/usr/local/include/opencv2)
include_directories(/usr/local/include/opencv)
link_directories(/usr/local/lib)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#####################################################
#            Use external libraries                 #
#find_package(Qt4 REQUIRED)
#####################################################

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

include_directories(/home/robot/ros_workspace/hg/eigen/include)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

rosbuild_add_executable(MotorControl src/MotorControl/MotorControl.cpp src/Amee.cpp)
rosbuild_add_executable(IRSensorReader src/IRSensor/IRSensorReader.cpp src/Amee.cpp)
rosbuild_add_executable(MovementControl src/MovementControl/MovementControl.cpp src/MovementControl/MoveRotate.cpp src/MovementControl/MoveStraight.cpp src/MovementControl/MoveStop.cpp src/MovementControl/MoveFollowWall.cpp src/MovementControl/MoveAlignWall.cpp src/Amee.cpp) 
rosbuild_add_executable(MeasureAmee src/MeasureAmee/MeasureAmee.cpp src/Amee.cpp)
rosbuild_add_executable(KeyHandler src/KeyHandler/key_handler.cpp)
rosbuild_add_executable(KeyHandlerTester src/KeyHandler/key_handler_listener.cpp)
rosbuild_add_executable(PublishAmee src/PublishAmee/PublishAmee.cpp)
rosbuild_add_executable(Localize src/Localize/Localize.cpp src/Localize/EKF.cpp src/Amee.cpp)
rosbuild_add_executable(IMUReader src/IMUSensor/IMUReader.cpp src/Amee.cpp)
rosbuild_add_executable(Mapper src/Mapper/Mapper.cpp src/Amee.cpp)
#rosbuild_add_executable(TagDetector src/Camera/TagDetector.cpp)
#target_link_libraries(TagDetector opencv_core opencv_highgui opencv_imgproc opencv_video)

#rosbuild_add_executable(CameraNode src/Camera/CameraNode.cpp)
#target_link_libraries(CameraNode v4l2)
#target_link_libraries(CameraNode opencv_core opencv_highgui)

#target_link_libraries(example ${PROJECT_NAME})
