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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

include_directories(
  ${PROJECT_SOURCE_DIR}/thirdparty/libcvd/include 
)  

link_directories(
  ${PROJECT_SOURCE_DIR}/thirdparty/libcvd/lib 
)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(testRPY src/testQuatToRPY.cpp)
rosbuild_add_executable(testTF src/testTF.cpp)
rosbuild_add_executable(testPTAM src/testPTAM.cpp)
<<<<<<< HEAD
rosbuild_add_executable(state_estimation src/stateestimation/main_stateestimation.cpp src/stateestimation/DroneKalmanFilter.cpp src/stateestimation/EstimationNode.cpp)
=======
rosbuild_add_executable(state_estimation src/stateestimation/main_stateestimation.cpp src/stateestimation/DroneKalmanFilter.cpp src/stateestimation/EstimationNode.cpp src/stateestimation/ScaleMap.cpp)
>>>>>>> 0339d1aef307420878e1cce4833a127c3740ab07
rosbuild_add_executable(autopilot src/autopilot/main_autopilot.cpp src/autopilot/DroneController.cpp src/autopilot/ControlNode.cpp)
rosbuild_add_executable(circle src/circle/main_circle.cpp src/circle/Circle.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# UI Stuff

ADD_DEFINITIONS( -Wall )
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
QT4_ADD_RESOURCES(GUI_RESOURCE_FILES_CPP ${GUI_RESOURCE_FILES})
QT4_WRAP_UI(GUI_UI_FILES_HPP ${GUI_UI_FILES})
QT4_WRAP_CPP(GUI_HEADER_FILES_HPP ${GUI_HEADER_FILES})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

rosbuild_add_executable(UI src/UI/main_UI.cpp src/UI/Rosthread.cpp src/UI/Teleop.cpp)
rosbuild_add_executable(interact src/UI/main_Interact.cpp src/UI/Interact.cpp src/UI/ThreadROS.cpp)
target_link_libraries(UI ${QT_LIBRARIES} cvd)
target_link_libraries(interact ${QT_LIBRARIES} cvd)



