from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    user = LaunchConfiguration("user")
    password = LaunchConfiguration("password")
    protocol = LaunchConfiguration("protocol")
    laser_frame_id = LaunchConfiguration("laser_frame_id")
    map_frame = LaunchConfiguration("map_frame")
    base_frame = LaunchConfiguration("base_frame")
    max_linear_speed_mps = LaunchConfiguration("max_linear_speed_mps")
    max_angular_speed_rad_s = LaunchConfiguration("max_angular_speed_rad_s")
    drive_throttle_pct = LaunchConfiguration("drive_throttle_pct")
    unsafe_drive = LaunchConfiguration("unsafe_drive")

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="192.168.1.1"),
            DeclareLaunchArgument("port", default_value="7272"),
            DeclareLaunchArgument("user", default_value="admin"),
            DeclareLaunchArgument("password", default_value="admin"),
            DeclareLaunchArgument("protocol", default_value="6MTX"),
            DeclareLaunchArgument("laser_frame_id", default_value="base_link"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("max_linear_speed_mps", default_value="0.5"),
            DeclareLaunchArgument("max_angular_speed_rad_s", default_value="1.0"),
            DeclareLaunchArgument("drive_throttle_pct", default_value="100.0"),
            DeclareLaunchArgument("unsafe_drive", default_value="true"),
            Node(
                package="ros_omron_agv",
                executable="omron_laser_node",
                name="omron_lasers",
                output="screen",
                parameters=[
                    {
                        "host": host,
                        "port": port,
                        "user": user,
                        "password": password,
                        "protocol": protocol,
                        "frame_id": laser_frame_id,
                    }
                ],
            ),
            Node(
                package="ros_omron_agv",
                executable="robot_status_node",
                name="robot_status",
                output="screen",
                parameters=[
                    {
                        "host": host,
                        "port": port,
                        "user": user,
                        "password": password,
                        "protocol": protocol,
                        "map_frame": map_frame,
                        "base_frame": base_frame,
                        "max_linear_speed_mps": max_linear_speed_mps,
                        "max_angular_speed_rad_s": max_angular_speed_rad_s,
                        "drive_throttle_pct": drive_throttle_pct,
                        "unsafe_drive": unsafe_drive,
                    }
                ],
            ),
            Node(
                package="ros_omron_agv",
                executable="map_node",
                name="map_server",
                output="screen",
                parameters=[
                    {
                        "host": host,
                        "port": port,
                        "user": user,
                        "password": password,
                        "protocol": protocol,
                        "frame_id": map_frame,
                    }
                ],
            ),
        ]
    )