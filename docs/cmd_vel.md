## cmd_vel Interface

`robot_status_node` subscribes to `/cmd_vel` as a standard ROS 2
`geometry_msgs/msg/Twist` stream.

The Omron ArNetworking server does not accept `Twist` commands directly. It uses
the `ratioDrive` request, where translational and rotational commands are sent
as percentages of the server's configured limits.

That means there is an explicit conversion layer in this package.

## What The Node Receives

The ROS side uses physical units:

- `linear.x` in meters per second
- `angular.z` in radians per second

Example keyboard teleop values:

- `linear.x = 0.50`
- `angular.z = 1.00`

## What The Robot Server Expects

The Omron server request `ratioDrive` expects:

- translational percentage
- rotational percentage
- overall throttle percentage
- lateral percentage

So the node converts from SI units into percentages before calling
`ArClientRatioDrive`.

## Conversion Used By This Package

The current node computes:

```text
trans_pct = clamp((linear_x_mps / max_linear_speed_mps) * 100, -100, 100)
rot_pct   = clamp((angular_z_rad_s / max_angular_speed_rad_s) * 100, -100, 100)
```

Where:

- `max_linear_speed_mps` is the linear speed mapped to `100%`
- `max_angular_speed_rad_s` is the angular speed mapped to `100%`

The current launch defaults are:

- `max_linear_speed_mps = 1.2`
- `max_angular_speed_rad_s = 1.5`
- `drive_throttle_pct = 100.0`
- `unsafe_drive = true`

With those values:

- `0.50 m/s -> 41.7%`
- `1.00 rad/s -> 66.7%`

That matches the current runtime logs.

## Runtime Feedback Logging

During teleop, `robot_status_node` logs:

- requested linear and angular velocity in ROS units
- converted translational and rotational percentages
- measured robot feedback speed from `updateNumbers`

Example:

```text
teleop req linear=0.50 m/s angular=0.00 rad/s -> trans=41.7% rot=0.0% | feedback trans=0.28 m/s lat=0.00 m/s rot=0.0 deg/s
```

Interpretation:

- `0.50 m/s` is what ROS requested
- `41.7%` is what was sent to the server
- `0.28 m/s` is what the robot reported back while moving

The rotational feedback is reported by the robot in degrees per second.

## How To Tune The Max Speed Parameters

Tune these values to match the robot-side limits configured in Mobile Planner or
the robot server.

### Option 1: Use Known Server Limits

If you know the server's configured maximum translational and rotational speeds,
set those values directly in launch.

Example:

```bash
ros2 launch ros_omron_agv omron_bringup.launch.py \
	max_linear_speed_mps:=0.9 \
	max_angular_speed_rad_s:=1.2
```

If the server maximum translation is `0.9 m/s`, then `linear.x = 0.45` becomes
`50%`.

### Option 2: Infer Limits From Feedback

If you do not know the server limits, estimate them from the live log.

Formula:

```text
estimated_100pct_speed = measured_speed / (sent_percent / 100)
```

Example from a log line:

```text
trans=41.7% | feedback trans=0.28 m/s
```

Then:

```text
estimated_100pct_linear = 0.28 / 0.417 = 0.67 m/s
```

That suggests the server is effectively limiting translational speed to about
`0.67 m/s` under the current conditions.

Do the same for rotation using the reported `rot` percentage and measured
rotational feedback.

### Option 3: Use The CLI First

The CLI is useful for controlled calibration.

Build and source:

```bash
cd /home/ubuntu/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select libaria
source install/setup.bash
```

Then test small ratio commands:

```bash
./install/libaria/bin/omron_robot_cli -host 192.168.1.1 -p 7272 -u admin -pw admin
```

At the prompt:

```text
unsafe
ratio 10 0 1000
watch 6 250
```

Use the reported feedback speed to estimate the real `100%` value.

## Recommended Tuning Procedure

1. Start in an open area.
2. Enable `unsafe` only if you understand the risk and need direct teleop.
3. Send a small command such as `linear.x = 0.2`.
4. Watch the feedback speed in the log.
5. Adjust `max_linear_speed_mps` and `max_angular_speed_rad_s` until the
	 conversion reflects the robot's real configured limits.
6. Re-test forward, reverse, and rotation separately.

## Important Practical Note

The values sent to the Omron server are percentages, not physical units.
The ROS interface remains in physical units because this package performs the
conversion before transmitting the command.
