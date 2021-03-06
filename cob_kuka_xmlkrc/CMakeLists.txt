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

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/src)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# for every node that is executable we have to generate a line like:
rosbuild_add_executable(cob_kuka_xmlkrc ros/src/cob_kuka_xmlkrc_ros.cpp) 

# protected region user Cmake macros on begin #
rosbuild_add_library(pugixml_lib common/src/pugixml.cpp)
rosbuild_add_library(KukaEthernetClient_lib common/src/KukaEthernetClient.cpp common/src/KukaAxis.cpp common/src/KukaFrame.cpp common/src/StringEthernetClient.cpp common/src/XmlStringHandler.cpp)
target_link_libraries(cob_kuka_xmlkrc KukaEthernetClient_lib pugixml_lib PocoNet)
# protected region user Cmake macros end #
