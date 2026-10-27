cmake_minimum_required(VERSION 3.16)
project(advrf_ros2_io)

find_package(ament_cmake REQUIRED)
foreach(dependency rclcpp sensor_msgs std_msgs advrf_ros2_interfaces)
  find_package(${dependency} REQUIRED)
endforeach()

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

add_executable(joint_control_node src/nodes/joint_control_node.cpp)
target_link_libraries(joint_control_node joint_control_client)
target_compile_options(joint_control_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(joint_control_node rclcpp sensor_msgs std_msgs advrf_ros2_interfaces)

add_executable(joy_control_node src/nodes/joy_control_node.cpp)
target_compile_features(joy_control_node PRIVATE cxx_std_17)
target_compile_options(joy_control_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(joy_control_node rclcpp sensor_msgs std_msgs advrf_ros2_interfaces)

install(TARGETS joint_control_node joy_control_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS joint_control joint_control_client EXPORT export_joint_control
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_joint_control HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs advrf_ros2_interfaces)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_gtest(test_joint_control test/test_joint_control.cpp)
  target_link_libraries(test_joint_control joint_control)
  ament_add_test(test_joint_control_node
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_joint_control_node.py" "$<TARGET_FILE:joint_control_node>" "$<TARGET_FILE:joy_control_node>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 30)
  ament_add_test(test_joint_control_split_axes
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_joint_control_node.py" "$<TARGET_FILE:joint_control_node>" "$<TARGET_FILE:joy_control_node>" "--split"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 30)
  ament_add_test(test_joint_discovery
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_joint_control_node.py" "$<TARGET_FILE:joint_control_node>" "$<TARGET_FILE:joy_control_node>" "--discover" "--split"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 30)
  ament_add_test(test_joint_control_speed_axis
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_joint_control_node.py" "$<TARGET_FILE:joint_control_node>" "$<TARGET_FILE:joy_control_node>" "--discover" "--speed"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 30)
  ament_add_test(test_joint_control_triggers
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_joint_control_node.py" "$<TARGET_FILE:joint_control_node>" "$<TARGET_FILE:joy_control_node>" "--discover" "--triggers"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 30)
endif()
ament_package()
