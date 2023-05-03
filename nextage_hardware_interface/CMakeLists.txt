cmake_minimum_required(VERSION 2.8.3)
project(nextage_hardware_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    controller_manager
    hardware_interface
    combined_robot_hw
    std_msgs
    std_srvs
    streaming_controllers
)

find_package(PkgConfig REQUIRED)

pkg_check_modules(ORB REQUIRED omniORB4)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
    include
    include/nextage_hardware_interface/
    include/kspapi
    ${CORBA_GENERATE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${ORB_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} 
    src/kspapi/jointAnglePlugin.cc
	src/kspapi/HRPcontroller.cc
	src/kspapi/OpenHRPCommon.cc
	src/kspapi/statusPlugin.cc
	src/kspapi/ViewSimulator.cc
	src/kspapi/servoPlugin.cc
	src/kspapi/dioPlugin.cc
    src/NextageAHardwareInterface.cpp
)

## Add cmake target dependencies of the library
add_dependencies(
    ${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
    ${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_LIBRARIES}
    ${ORB_LIBRARIES}
)

## Declare a C++ executable
add_executable(NextageAHardwareInterface_node src/NextageAHardwareInterface_node.cpp)
target_link_libraries(NextageAHardwareInterface_node ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} NextageAHardwareInterface_node
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(FILES nextage_hw_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
