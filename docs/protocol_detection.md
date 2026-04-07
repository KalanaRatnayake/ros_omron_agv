## Protocol Detection And Interface Inspection

This package currently connects to the Omron robot server on TCP port `7272`
using ArNetworking protocol `6MTX`.

That value is not guaranteed forever. If Omron changes the server protocol in a
future firmware release, the quickest way to recover is to separate two
questions:

1. Which protocol string does the server accept?
2. Which requests does the server advertise after connection?

This document shows how to answer both.

## Fastest Check: Use The CLI

Build and source the workspace:

```bash
cd /home/ubuntu/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select libaria
source install/setup.bash
```

Then connect with interface inspection enabled:

```bash
./install/libaria/bin/omron_robot_cli \
	-host 192.168.1.1 \
	-p 7272 \
	-u admin \
	-pw admin \
	--check-interface
```

What this does:

- Tries `6MTX`, then `D6MTX`, then `5MTX`
- Connects with the first protocol the server accepts
- Prints the full advertised request list after connection

If the connection succeeds, note two things:

- The selected protocol printed at startup
- The request names printed by `--check-interface`

That is usually enough to adapt this package to a new controller.

## Disable Protocol Enforcement Temporarily

If you want the client to accept whatever protocol the server advertises, pass
an empty protocol string:

```bash
./install/libaria/bin/omron_robot_cli \
	-host 192.168.1.1 \
	-p 7272 \
	-u admin \
	-pw admin \
	-protocol "" \
	--check-interface
```

Use this when you suspect the robot has moved to a newer protocol string but
the rest of the interface is still compatible.

If this works while `-protocol 6MTX` does not, the next step is to capture the
server handshake or update the fallback list.

## Identify The Robot Interface

The advertised request list tells you what the robot actually supports. In this
workspace the useful requests include items such as:

- `ratioDrive`
- `setSafeDrive`
- `gotoPose`
- `dock`
- `undock`
- `updateNumbers`
- `updateStrings`
- `dockInfoChanged`
- `Laser_1Current`

The ROS 2 nodes depend on those requests as follows:

- `robot_status_node`
	uses `updateNumbers`, `updateStrings`, `dockInfoChanged`, `ratioDrive`, `setSafeDrive`, `gotoPose`, `localizeToPose`, `dock`, `stop`
- `omron_laser_node`
	uses `Laser_1Current` and optionally `Laser_2Current`
- `map_node`
	uses `getMapName` and `getMap`

If any of those names change on a future robot, update the node code or expose
the new request names as parameters.

## Raw TCP Handshake Inspection

If the server stops accepting known protocol strings, capture the first bytes on
the TCP connection before any higher-level parsing.

### Python Example

This script opens the TCP socket and prints any bytes immediately sent by the
server. Some controllers send version or banner data as soon as the connection
is accepted.

```python
#!/usr/bin/env python3
import socket

host = "192.168.1.1"
port = 7272

with socket.create_connection((host, port), timeout=5.0) as sock:
		sock.settimeout(2.0)
		try:
				data = sock.recv(512)
				print("received", len(data), "bytes")
				print(data)
				print(data.hex())
		except socket.timeout:
				print("no immediate banner received")
```

Run it with:

```bash
python3 detect_protocol.py
```

If the server does not send an immediate banner, use a packet capture during a
successful CLI connection.

### Packet Capture Example

```bash
sudo tcpdump -i any -s 0 -X host 192.168.1.1 and port 7272
```

Then in another terminal:

```bash
source /home/ubuntu/colcon_ws/install/setup.bash
./install/libaria/bin/omron_robot_cli -host 192.168.1.1 -p 7272 -u admin -pw admin --check-interface
```

Compare captures between working and failing protocol attempts.

## Updating The Code For A New Protocol

There are three places to look.

1. CLI fallback list in [omron_robot_cli.cpp](../../libaria/tools/omron_robot_cli.cpp)
2. ROS 2 launch default `protocol` in [omron_bringup.launch.py](../launch/omron_bringup.launch.py)
3. Any deployment scripts or launch files that override `protocol`

If the server still accepts `6MTX`, keep it as the default. Only change the
default once you have verified a newer protocol is required.

## Minimal Recovery Workflow For Future Firmware Changes

1. Run the CLI with `--check-interface` and the normal `6MTX` default.
2. Retry with `-protocol ""`.
3. Capture the handshake with Python or `tcpdump` if both fail.
4. Record the new protocol string and the advertised request list.
5. Update the fallback list and the ROS 2 launch default.
6. Re-test `status`, `ratio`, `cmdvel`, `goto`, docking, laser, and map.
