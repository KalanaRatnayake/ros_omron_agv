## Laser Data And Conversion

`omron_laser_node` currently publishes Omron laser data as
`sensor_msgs/msg/PointCloud2`, not `sensor_msgs/msg/LaserScan`.

This is intentional: the Omron `Laser_1Current` and `Laser_2Current` requests
arrive as Cartesian point samples, so the direct lossless ROS representation is
a point cloud.

## What The Robot Sends

For each laser update packet the node reads:

1. `num_readings` as a 4-byte integer
2. `x` for each point as a 4-byte integer in millimeters
3. `y` for each point as a 4-byte integer in millimeters

The node converts each sample to meters and publishes:

- `x = packet_x_mm / 1000.0`
- `y = packet_y_mm / 1000.0`
- `z = 0.0`

Those points are stored in a `PointCloud2` message with fields:

- `x`
- `y`
- `z`

## Topics Published By Default

- `/laser` from `Laser_1Current`
- `/laser_low` from `Laser_2Current`, if the robot advertises it

The current robot only advertises `Laser_1Current`, so `/laser_low` may not be
present.

## Why This Is Not Already LaserScan

`LaserScan` assumes a single ordered radial sweep with uniform angular spacing.
The Omron network packet used here does not expose those scan metadata values
directly.

You can still derive a `LaserScan` if the point set represents a single planar
scanner and you choose:

- a target frame
- an angular binning strategy
- a range min and max
- how to handle multiple points in the same angular bin

## Basic Conversion Rules

For each point `(x, y)` in meters:

```text
range = sqrt(x^2 + y^2)
angle = atan2(y, x)
```

Then place the point into a `LaserScan.ranges[index]` bin using:

```text
index = floor((angle - angle_min) / angle_increment)
```

If multiple points land in the same bin, keep the nearest range.

## Python Example: PointCloud2 To LaserScan

The following ROS 2 Python node subscribes to the point cloud and republishes a
derived `LaserScan`.

```python
#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


class CloudToScan(Node):
	def __init__(self):
		super().__init__("cloud_to_scan")
		self.angle_min = -math.pi
		self.angle_max = math.pi
		self.angle_increment = math.radians(0.5)
		self.range_min = 0.02
		self.range_max = 30.0
		self.frame_id = "base_link"

		self.subscription = self.create_subscription(
			PointCloud2, "/laser", self.handle_cloud, 10
		)
		self.publisher = self.create_publisher(LaserScan, "/scan", 10)

	def handle_cloud(self, msg: PointCloud2):
		bins = int(math.ceil((self.angle_max - self.angle_min) / self.angle_increment))
		ranges = [float("inf")] * bins

		for x, y, z in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
			rng = math.hypot(x, y)
			if rng < self.range_min or rng > self.range_max:
				continue

			angle = math.atan2(y, x)
			if angle < self.angle_min or angle > self.angle_max:
				continue

			index = int((angle - self.angle_min) / self.angle_increment)
			if 0 <= index < bins:
				ranges[index] = min(ranges[index], rng)

		scan = LaserScan()
		scan.header = msg.header
		scan.header.frame_id = self.frame_id
		scan.angle_min = self.angle_min
		scan.angle_max = self.angle_max
		scan.angle_increment = self.angle_increment
		scan.range_min = self.range_min
		scan.range_max = self.range_max
		scan.ranges = [r if math.isfinite(r) else float("nan") for r in ranges]
		self.publisher.publish(scan)


def main():
	rclpy.init()
	node = CloudToScan()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
	main()
```

## Practical Notes

- Use the robot laser frame if you know it, otherwise keep the current frame
  and transform downstream.
- If the point cloud orientation looks mirrored or rotated in RViz, check the
  robot frame convention before changing the conversion code.
- If you need a production-quality `LaserScan`, add filtering for NaNs,
  outliers, and bins with no returns.

## Verifying The Data

Inspect the point cloud first:

```bash
ros2 topic echo /laser --once
```

Or visualize it in RViz using `PointCloud2`.

Only convert to `LaserScan` once the cloud orientation and frame assignment are
confirmed.
