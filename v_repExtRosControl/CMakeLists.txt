cmake_minimum_required(VERSION 2.8.3)
project(v_repExtRosControl)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_MACOSX_RPATH 1)
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
	joint_limits_interface
  roscpp
)

set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(VREP 3.5.0 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES v_repExtRosControl
#  CATKIN_DEPENDS controller_manager hardware_interface joint_limits_interface roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${VREP_INCLUDE})
include_directories(${CMAKE_CURRENT_BINARY_DIR}/generated)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external)
include_directories(${Boost_INCLUDE_DIRS})

vrep_generate_stubs(${CMAKE_CURRENT_BINARY_DIR}/generated
	XML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/callbacks.xml
	LUA_FILE ${CMAKE_CURRENT_SOURCE_DIR}/simExtRosControl.lua
)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in
	${CMAKE_CURRENT_BINARY_DIR}/config.h ESCAPE_QUOTES)

set(SOURCES
    src/hardware_interface_vrep/robot_hw_vrep.cpp
    src/hardware_interface_vrep/robot.cpp
		src/v_repExtRosControl.cpp
    external/v_repPlusPlus/Plugin.cpp
)

add_library(${PROJECT_NAME}
	SHARED
		${SOURCES}
		${CMAKE_CURRENT_BINARY_DIR}/generated/stubs.cpp
		"${VREP_EXPORTED_SOURCES}"
)
target_link_libraries(${PROJECT_NAME}
	${Boost_LIBRARIES}
	${VREP_LIBRARIES}
	${catkin_LIBRARIES}
)


install(TARGETS ${PROJECT_NAME} DESTINATION ${VREP_ROOT})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/simExtRosControl.lua DESTINATION ${VREP_ROOT}/lua)
