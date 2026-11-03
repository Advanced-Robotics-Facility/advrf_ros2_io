# advrf_ros2_io

Event-driven control of one selected joint, using the ROS interfaces of
`advrf_ros2_motors_controller`.

## Run

Build with `colcon build --packages-up-to advrf_ros2_io`, then source
`install/setup.bash`. Run the motor power node and direct joint controller in
the same namespace. The IO node does not start them.
Use the controller's position mode for position jogging; stop other publishers
of `joint_commands` and disable trajectory generation when using the IO node.

```bash
ros2 run advrf_ros2_io manual_joint_controller_node --ros-args \
  -r __ns:=/robot
```

Adapt `config/joint_control.yaml` for motion settings and joystick layout; load it
with `--params-file /path/to/joint_control.yaml`.

When `joints` is omitted (the default), the node discovers joint names in the first
nonempty, valid `rx/joint_states` message containing finite positions. It waits
without sending commands until that message arrives. Empty names, duplicates,
invalid positions and mismatched array sizes are ignored. The discovered order
is retained, even if later feedback messages reorder or omit joints; missing
feedback is handled by the freshness check.

Do not write `joints: []` in a ROS YAML parameter file: an empty array has no
element type. Leave the parameter out for discovery.

To restrict selection, explicitly set `joints`, e.g.
`-p 'joints:=[joint_a,joint_b]'` with `ros2 run`, or
`'joints:=[joint_a,joint_b]'` with the joystick launcher. Explicit names are
required when configuring position-limit arrays, so their ordering is known.
The first joint is selected initially. Selection wraps around in both directions.
Changing selection cancels continuous movement and resets its position target.
Nothing moves or powers on at startup.

## Joystick launcher

```bash
ros2 launch advrf_ros2_io joy.launch.py namespace:=/robot
```

Starts `joy/joy_node`, `joystick_input_node` and `manual_joint_controller_node` in the controller namespace.
The motor power and direct joint controller nodes must already be running.
Use `device_id:=1` to choose another SDL joystick, or `config:=/path/to/config.yaml`
to load motion settings. This launcher overrides the bindings with:

| Action | Input |
| --- | --- |
| START selected motor | Button 0 |
| STOP selected motor | Button 1 |
| Position minus | Axis 2: +1 = stopped, -1 = maximum |
| Position plus | Axis 5: +1 = stopped, -1 = maximum |
| Previous joint | Button 4 |
| Next joint | Button 5 |

Both position axes rest at `+1`. Deflecting axis 2 toward `-1` decreases
position; deflecting axis 5 toward `-1` increases position. Each trigger maps
`(1 - value) / 2` to a speed up to `axis_speed`, with the configured deadzone.
Equal trigger deflections cancel. Release both triggers after changing joint or
issuing START/STOP. This is configured with `position_trigger_axes: true`;
there is no separate speed axis in this mapping.
The driver repeats messages at 50 Hz so holding an axis keeps jogging.
The joystick adapter applies the deadzone; the driver's deadzone is disabled.

## Events and bindings

Any keyboard, GUI or input adapter can publish `advrf_ros2_interfaces/msg/JointControlEvent` to
`joint_control/events`:

| Event | Action |
| --- | --- |
| `NEXT=1`, `PREVIOUS=2` | Select the next/previous joint |
| `POWER_ON=3`, `POWER_OFF=4` | Start/stop the selected motor |
| `POSITION_PLUS=5`, `POSITION_MINUS=6` | Increment/decrement its target by `position_step` |
| `SYNC=7` | Stop jogging and reset the target to feedback on the next movement |

```bash
ros2 topic pub --once /robot/joint_control/events advrf_ros2_interfaces/msg/JointControlEvent '{action: 1}'
ros2 topic pub --once /robot/joint_control/events advrf_ros2_interfaces/msg/JointControlEvent '{action: 3}'
ros2 topic pub --once /robot/joint_control/events advrf_ros2_interfaces/msg/JointControlEvent '{action: 5}'
ros2 topic pub --once /robot/joint_control/events advrf_ros2_interfaces/msg/JointControlEvent '{action: 4}'
```

For a continuous axis, publish `std_msgs/msg/Float64` in `[-1, 1]` to
`joint_control/axis` faster than `input_timeout`. Send zero first to arm the
axis, and zero to release it. This is a normalized command: its amplitude
maps proportionally to `axis_speed` position units per second. Input adapters
apply device-specific deadzones and speed scaling before publishing.

The separate `joystick_input_node` subscribes to `sensor_msgs/msg/Joy` on `joy`
and publishes typed events and normalized axes to `manual_joint_controller_node`.
The controller has no joystick dependency in its input handling; a keyboard or
GUI adapter can publish the same two topics. The launch file starts both nodes.
Configuration is scoped separately to each node.

Joystick parameters:

| Parameter | Default index |
| --- | --- |
| `next_button` / `previous_button` | 5 / 4 |
| `power_on_button` / `power_off_button` | 0 / 1 |
| `position_plus_button` / `position_minus_button` | -1 / -1 (disabled) |
| `position_minus_axis` / `position_plus_axis` | 2 / 5 (configured) |
| `position_trigger_axes` | true (configured) |
| `position_axis` | -1 (disabled in this mapping) |
| `speed_axis` | -1 (disabled) |

Set an index to `-1` to disable it. Buttons trigger on rising edges. The first
message after startup/reconnection seeds button state without triggering actions.
When multiple buttons rise together, OFF wins; otherwise selection, ON, then
position steps take priority. Release and press again for the next event.
The axis must pass through neutral after startup, reconnection, selection or a
power event. `axis_deadzone` defaults to `0.15`; `axis_scale` is `1` or `-1` (configured as `1`).
`speed_axis` optionally selects a separate Joy speed multiplier in `[0, 1]`;
missing or invalid speed values stop jogging. The Float64 input remains a signed
axis without a separate speed multiplier.
Use one input adapter at a time: inputs share the same selected joint and target.

## ROS interfaces

| Interface | Type | Purpose |
| --- | --- | --- |
| `rx/joint_states` subscription | `sensor_msgs/msg/JointState` | Actual joint positions |
| `joint_commands` publisher | `sensor_msgs/msg/JointState` | Only the selected joint's position |
| `motor_power_request` client | `advrf_ros2_interfaces/srv/MotorPowerRequest` | ON/OFF for the selected joint |
| `selected_joint` publisher | `sensor_msgs/msg/JointState` | Selected name, retained for new subscribers |

Names are relative and can be remapped. Power requests use the controller's
configured mode and START gains. A request captures the joint selected when
it was triggered, even if selection changes while waiting for the response.
Power calls are asynchronous, with no automatic retries. Errors and unknown
outcomes appear in the node log. Movement is suspended while a power request
is pending and the position target is reset when it completes.

Position steps use feedback initially, then accumulate a target until reset.
Axis values integrate at `publish_rate` (default 50 Hz), at `axis_speed` (0.1 by
default). `position_step` defaults to 0.01. Units follow the controller: radians
for revolute joints, meters for prismatic joints.
Movement requires fresh selected-joint feedback (`feedback_timeout`, default
0.5 s) and a `joint_commands` subscriber. Missing axis input (`input_timeout`,
default 0.25 s) or a delayed timer stops integration; this does not send motor
STOP. Optional paired `lower_positions` / `upper_positions` arrays clamp targets;
the existing controller still validates commands against its own joint limits.

## Reuse in C++

`JointControl` is independent of ROS and input devices. It accepts position and
power callbacks, feedback, `event(JointEvent)` and `axis(double)` calls, and a
periodic `tick(dt)`. `JointControlClient` supplies its ROS bridge:

```cpp
#include <advrf_ros2_io/joint_control_client.hpp>

// As a member of your node; call from a single-threaded executor.
auto control = std::make_unique<advrf_ros2_io::JointControlClient>(*this);
control->event(advrf_ros2_io::JointEvent::Next);
control->event(advrf_ros2_io::JointEvent::PowerOn);
control->event(advrf_ros2_io::JointEvent::PositionPlus);
// A joystick callback can call control->axis(value).
```

Keep the client alive for the node's lifetime. `ready()` indicates whether
joints are available; `selected_joint()` returns an empty string until discovery.
Events before discovery are rejected and axis inputs are ignored. Declare your ROS dependency on
`advrf_ros2_io` and link its exported `advrf_ros2_io::joint_control_client` target. The generic
core is exported separately as `advrf_ros2_io::joint_control`.

## Tests

`colcon test --packages-select advrf_ros2_io` runs core tests and a mock ROS
integration test for selection, power, position, joystick edges and watchdogs.
The mock test runs in an isolated ROS domain and sends no hardware commands.


Grouped launch topic paths: `<robot>/controllers/joint_commands` for joint
commands and `<robot>/inputs/selected_joint` for input selection. The panel
uses the grouped command topic. Actions and power services keep their existing
robot-level paths. Standalone nodes use relative topics; apply equivalent
remappings when running them outside the supplied launch files.
