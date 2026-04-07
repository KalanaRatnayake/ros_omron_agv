# OMRON ROS 2 Robot Driver

The OMRON LD-60 is a capable platform out of the box but has no ROS support.
This package is being migrated to ROS 2 and currently provides ROS 2 Jazzy
laser, robot status, and map nodes built on top of `libaria`.

This does not replace Mobile Planner. Mobile Planner is still used for map
creation and robot configuration.

## Migration Status

Currently ported to ROS 2:

- Package build system (`ament_cmake` + `rosidl`)
- `omron_laser_node`
- `robot_status_node`
- `map_node`
- ROS 2 launch file
- Custom message and service generation

Still pending ROS 2 refinement:

- Nav2 action integration for higher-level autonomous goal handling
- Final interface review against your target ROS 2 topic and service contract

<img src="./docs/omron_robot.jpg" alt="LD-60 Robot" style="zoom:33%;" />

## Required Parameters

 **Host IP**: String     e.g. 172.168.1.1

 **Host Port**: String     e.g. 7272

 **User**: String  e.g. omron



## Topics 

### Published

* /laser
* /laser_low
* /current_pose
* /robot_status
* /map
* /map_metadata

### Subscribed

* /cmd_vel
* /goal_pose
* /initialpose

### Services

* /dock
* /get_map

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd /home/ubuntu/colcon_ws
colcon build --packages-select libaria ros_omron_agv
source /home/ubuntu/colcon_ws/install/setup.bash
```

## Run

Launch the ROS 2 bringup:

```bash
ros2 launch ros_omron_agv omron_bringup.launch.py \
	host:=192.168.1.1 \
	port:=7272 \
	user:=admin \
	password:=admin \
	protocol:=6MTX
```

The launch file starts `omron_laser_node`, `robot_status_node`, and `map_node`.

## Parameters

- `host`: robot IP or hostname
- `port`: ArNetworking port, typically `7272`
- `user`: robot user name
- `password`: robot password, or empty string for no password
- `protocol`: enforced ArNetworking protocol, default `6MTX`
- `laser_frame_id`: frame id for published point clouds
- `map_frame`: frame id for map publication and the map side of the robot transform
- `base_frame`: child frame id for the robot transform
- `max_linear_speed_mps`: linear speed that maps to `100%` ratioDrive, default `0.5`
- `max_angular_speed_rad_s`: angular speed that maps to `100%` ratioDrive, default `1.0`
- `drive_throttle_pct`: overall ratioDrive throttle percentage, default `100.0`
- `unsafe_drive`: whether teleop should request unsafe drive on startup, default `true`
- `primary_request`: primary laser request name, default `Laser_1Current`
- `secondary_request`: secondary laser request name, default `Laser_2Current`
- `primary_topic`: topic for the primary laser, default `/laser`
- `secondary_topic`: topic for the secondary laser, default `/laser_low`
- `request_period_ms`: server request interval in milliseconds



## Getting Started

1. Verify the robot accepts the chosen credentials and protocol with `omron_robot_cli`.
2. Start the ROS 2 launch file.
3. Inspect `/laser`, `/current_pose`, `/robot_status`, and `/map` using `ros2 topic echo` or RViz.
