cmake_minimum_required(VERSION 3.16)
project(touch_driver CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(MSVC)
  add_compile_options(/W3 /utf-8)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(urdf REQUIRED)

# ---- CHAI3D (not a ROS package). Default: the chai3d-3.3.0 folder next to this workspace.
#      Override with the CHAI3D_ROOT environment variable. It must be built in Release (it is: build/Release).
set(CHAI3D_ROOT "$ENV{CHAI3D_ROOT}")
if(NOT CHAI3D_ROOT)
  get_filename_component(CHAI3D_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../chai3d-3.3.0" ABSOLUTE)
endif()
file(TO_CMAKE_PATH "${CHAI3D_ROOT}" CHAI3D_ROOT)
find_package(CHAI3D CONFIG REQUIRED NAMES chai3d PATHS "${CHAI3D_ROOT}/build" NO_DEFAULT_PATH)
set(GLFW_LIB "${CHAI3D_ROOT}/build/extras/GLFW/Release/glfw.lib")
set(HDPHANTOM_DLL "${CHAI3D_ROOT}/bin/win-x64/hdPhantom64.dll")
foreach(f "${GLFW_LIB}" "${HDPHANTOM_DLL}")
  if(NOT EXISTS "${f}")
    message(FATAL_ERROR "CHAI3D file missing: ${f} (build CHAI3D in Release first)")
  endif()
endforeach()

# ---- kinematics: URDF -> planar SO-101 model. No Eigen, no CHAI3D.
add_library(so101_kinematics STATIC src/kinematics.cpp)
target_include_directories(so101_kinematics PUBLIC include)
target_link_libraries(so101_kinematics PUBLIC urdf::urdf)

# ---- CHAI3D side (device, haptic loop, window), compiled with CHAI3D's own settings.
#      Kept in its own library so CHAI3D's headers (and its bundled Eigen) never meet ROS headers.
add_library(touch_haptics STATIC src/haptics.cpp)
target_include_directories(touch_haptics PUBLIC include PRIVATE ${CHAI3D_INCLUDE_DIRS} "${CHAI3D_ROOT}/extras/GLFW/include")
# the same definitions CHAI3D itself was compiled with (see chai3d-3.3.0/CMakeLists.txt)
target_compile_definitions(touch_haptics PRIVATE WIN64 GLEW_STATIC _CRT_SECURE_NO_DEPRECATE)
target_link_libraries(touch_haptics PUBLIC so101_kinematics PRIVATE ${CHAI3D_LIBRARIES} "${GLFW_LIB}")

# ---- the node
add_executable(touch_node src/touch_node.cpp)
target_compile_definitions(touch_node PRIVATE NOMINMAX)
target_link_libraries(touch_node
  touch_haptics so101_kinematics
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS} ${std_msgs_TARGETS} ${std_srvs_TARGETS})

# ---- tests: no device, no ROS. Run them from install\lib\touch_driver\.
add_executable(test_haptic_math test/test_haptic_math.cpp)
target_include_directories(test_haptic_math PRIVATE include)

# test_kinematics compares our analytic FK/IK with KDL (only the TEST uses KDL)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(orocos_kdl REQUIRED)
add_executable(test_kinematics test/test_kinematics.cpp)
target_link_libraries(test_kinematics so101_kinematics kdl_parser::kdl_parser ${orocos_kdl_LIBRARIES})
target_include_directories(test_kinematics PRIVATE ${orocos_kdl_INCLUDE_DIRS})
get_filename_component(SO101_URDF "${CMAKE_CURRENT_SOURCE_DIR}/../so101_description/urdf/so101.urdf" ABSOLUTE)
target_compile_definitions(test_kinematics PRIVATE SO101_URDF="${SO101_URDF}")

if(BUILD_TESTING)
  enable_testing()
  add_test(NAME test_haptic_math COMMAND test_haptic_math)
  add_test(NAME test_kinematics COMMAND test_kinematics)
endif()

install(TARGETS touch_node test_haptic_math test_kinematics DESTINATION lib/${PROJECT_NAME})
# CHAI3D loads the Phantom/Touch driver at runtime with LoadLibrary("hdPhantom64.dll"), which looks
# next to the .exe first -- so ship it there. (HD.dll itself is in System32, from the Touch driver.)
install(FILES "${HDPHANTOM_DLL}" DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
