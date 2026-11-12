# advrf_ros2_io
ros version: jazzy

## Quick Use

```bash
ros2 launch advrf_ros2_bringup inputs.launch.py --show-args
```

For Miranda with the packaged input configuration:

```bash
ros2 launch advrf_ros2_miranda_bringup miranda.launch.py use_joy:=true
```

## Edit Config

Use `joint_control_config` for button mappings, axes, deadzone and input timeout. The shared launch also accepts explicit robot and controller configuration paths.

Joystick parameters include `position_axis`, `speed_axis`, `axis_deadzone`, `axis_scale`, `input_timeout` and `toggle_active_button`.

## Nodes Details

| Node | Purpose |
| --- | --- |
| `joystick_input_node` | Converts `joy` (`sensor_msgs/msg/Joy`) into manual control events and axis values |
| `manual_joint_controller_node` | Consumes manual events and axis values through the joint control client |

- Events: `joint_control/events` (`advrf_ros2_interfaces/msg/JointControlEvent`).
- Axis: `joint_control/axis` (`std_msgs/msg/Float64`).
- Joint power client: `advrf_ros2_interfaces/srv/MotorPowerRequest`.
- The package also provides RViz integration; use the robot launch for configured namespace and topic remappings.

## Dependencies

- `rclcpp`
- `pluginlib`
- `rviz_common`
- `rviz_default_plugins`
- `std_msgs`
- `std_srvs`
- `sensor_msgs`
- `advrf_ros2_interfaces`
- `advrf_ros2_motors_controller`
- `joy`
- `launch_ros`
- `launch`
- `rclpy`
- `visualization_msgs`
- `geometry_msgs`
- `ament_index_python`

## TroubleShooting

| Symptom | Check |
| --- | --- |
| No joystick events | `joy` topic, joystick device and button mappings |
| Axis has no effect | Axis index, deadzone, input timeout and selected joint |
| Power request fails | Power service availability and configured joint name |
