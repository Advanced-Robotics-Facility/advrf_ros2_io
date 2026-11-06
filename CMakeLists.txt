cmake_minimum_required(VERSION 3.16)
project(advrf_ros2_io)

find_package(ament_cmake REQUIRED)
foreach(dependency rclcpp sensor_msgs std_msgs std_srvs advrf_ros2_interfaces)
  find_package(${dependency} REQUIRED)
endforeach()

find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(selected_joint_robot SHARED
  src/rviz/selected_joint_robot.cpp
  include/advrf_ros2_io/selected_joint_robot.hpp)
set_target_properties(selected_joint_robot PROPERTIES AUTOMOC ON)
target_compile_features(selected_joint_robot PUBLIC cxx_std_17)
target_compile_options(selected_joint_robot PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(selected_joint_robot PRIVATE include)
ament_target_dependencies(selected_joint_robot rclcpp sensor_msgs pluginlib rviz_common rviz_default_plugins)
target_link_libraries(selected_joint_robot Qt5::Widgets)
pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml)
install(TARGETS selected_joint_robot LIBRARY DESTINATION lib)

add_library(joint_control src/lib/joint_control.cpp)
target_compile_features(joint_control PUBLIC cxx_std_17)
target_compile_options(joint_control PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(joint_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(joint_control_client src/lib/joint_control_client.cpp)
target_link_libraries(joint_control_client joint_control)
target_compile_options(joint_control_client PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(joint_control_client rclcpp sensor_msgs advrf_ros2_interfaces)

add_executable(manual_joint_controller_node src/nodes/manual_joint_controller_node.cpp)
target_link_libraries(manual_joint_controller_node joint_control_client)
target_compile_options(manual_joint_controller_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(manual_joint_controller_node rclcpp sensor_msgs std_msgs advrf_ros2_interfaces)

add_executable(joystick_input_node src/nodes/joystick_input_node.cpp)
target_compile_features(joystick_input_node PRIVATE cxx_std_17)
target_compile_options(joystick_input_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(joystick_input_node rclcpp sensor_msgs std_msgs std_srvs advrf_ros2_interfaces)

install(PROGRAMS scripts/selected_joint_marker_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS manual_joint_controller_node joystick_input_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS joint_control joint_control_client EXPORT export_joint_control
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_joint_control HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs advrf_ros2_interfaces)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_test(test_joystick_active
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_joystick_active.py" "$<TARGET_FILE:joystick_input_node>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 20)
  ament_add_test(test_selected_joint_marker
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_selected_joint_marker.py"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_gtest(test_joint_control test/test_joint_control.cpp)
  target_link_libraries(test_joint_control joint_control)
  ament_add_test(test_manual_joint_controller_node
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_manual_joint_controller_node.py" "$<TARGET_FILE:manual_joint_controller_node>" "$<TARGET_FILE:joystick_input_node>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 30)
  ament_add_test(test_joint_control_split_axes
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_manual_joint_controller_node.py" "$<TARGET_FILE:manual_joint_controller_node>" "$<TARGET_FILE:joystick_input_node>" "--split"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 30)
  ament_add_test(test_joint_discovery
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_manual_joint_controller_node.py" "$<TARGET_FILE:manual_joint_controller_node>" "$<TARGET_FILE:joystick_input_node>" "--discover" "--split"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 30)
  ament_add_test(test_joint_control_speed_axis
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_manual_joint_controller_node.py" "$<TARGET_FILE:manual_joint_controller_node>" "$<TARGET_FILE:joystick_input_node>" "--discover" "--speed"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 30)
  ament_add_test(test_joint_control_triggers
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_manual_joint_controller_node.py" "$<TARGET_FILE:manual_joint_controller_node>" "$<TARGET_FILE:joystick_input_node>" "--discover" "--triggers"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 30)
endif()
# Optional integration test: needs an X/OpenGL display and a dedicated ROS domain.
option(ADVRF_BUILD_RENDER_TEST "Build the hidden-scene RViz integration test" OFF)
if(ADVRF_BUILD_RENDER_TEST)
  add_executable(test_selected_joint_robot test/selected_joint_robot_render.cpp)
  target_include_directories(test_selected_joint_robot PRIVATE include)
  target_compile_features(test_selected_joint_robot PRIVATE cxx_std_17)
  ament_target_dependencies(test_selected_joint_robot rclcpp sensor_msgs pluginlib rviz_common rviz_default_plugins)
  target_link_libraries(test_selected_joint_robot selected_joint_robot Qt5::Widgets)
endif()
ament_package()
