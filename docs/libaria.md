## Using libaria In Other Projects

This workspace builds a modern CMake package for the local Omron-compatible
`libaria` fork.

The package exports CMake config files and imported targets so other CMake
projects can link against it directly.

## What Gets Exported

The `libaria` package exports:

- `libaria::Aria`
- `libaria::ArNetworking`

Use `libaria::ArNetworking` when you need ArNetworking client classes such as:

- `ArClientBase`
- `ArClientRatioDrive`
- `ArNetPacket`

`libaria::ArNetworking` pulls in the required core `Aria` linkage for typical
client use.

## Build And Install In This Workspace

```bash
cd /home/ubuntu/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select libaria
source install/setup.bash
```

After sourcing `install/setup.bash`, the installed package config and shared
libraries are available through the workspace environment.

## CMake Usage

In another CMake project in the same sourced environment:

```cmake
cmake_minimum_required(VERSION 3.16)
project(example_client)

find_package(libaria CONFIG REQUIRED)

add_executable(example_client main.cpp)
target_link_libraries(example_client PRIVATE libaria::ArNetworking)
```

If your environment is not already sourced, set `CMAKE_PREFIX_PATH` to include
the workspace install tree.

Example:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/home/ubuntu/colcon_ws/install
```

## Minimal Example

```cpp
#include <Aria/Aria.h>
#include <ArNetworking/ArNetworking.h>

int main()
{
	Aria::init();
	ArLog::init(ArLog::StdOut, ArLog::Normal);

	ArClientBase client;
	client.enforceProtocolVersion("6MTX");

	if (!client.blockingConnect("192.168.1.1", 7272, true, "admin", "admin"))
	{
		return 1;
	}

	client.runAsync();
	client.disconnect();
	Aria::shutdown();
	return 0;
}
```

## Header Usage

Typical includes used by this workspace are:

```cpp
#include <Aria/Aria.h>
#include <ArNetworking/ArNetworking.h>
#include <ArNetworking/ArClientRatioDrive.h>
```

The package exposes headers from both the core Aria library and the
ArNetworking layer.

## Runtime Environment

There are two runtime requirements to remember.

### 1. Shared Libraries

Source the workspace before running your application:

```bash
source /home/ubuntu/colcon_ws/install/setup.bash
```

That sets up the shared library search path.

### 2. ARIA Install Directory

ARIA expects to know where its support files are located. In this workspace the
ROS 2 nodes set the `ARIA` environment variable automatically at compile time
using the local source tree.

If you write a standalone application outside this package, either:

- export `ARIA` yourself before running it, or
- set it in code before `Aria::init()`

Example:

```bash
export ARIA=/home/ubuntu/colcon_ws/src/libaria
```

Or in C++:

```cpp
#include <cstdlib>

setenv("ARIA", "/home/ubuntu/colcon_ws/src/libaria", 0);
Aria::init();
```

## Using libaria In Another ROS 2 Package

In `CMakeLists.txt`:

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(libaria CONFIG REQUIRED)

add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp)
target_link_libraries(my_node PRIVATE libaria::ArNetworking)
```

If your node also wants to suppress the ARIA startup warning, add a compile
definition or set `ARIA` in the launch environment.

## When To Link Aria Only

Use `libaria::Aria` by itself when you only need the lower-level robot library
and do not need ArNetworking client or server classes.

Use `libaria::ArNetworking` when you are talking to the Omron server on port
`7272`.

## Related Files

- Export configuration: `src/libaria/CMakeLists.txt`
- ROS 2 package usage example: [CMakeLists.txt](../CMakeLists.txt)
- CLI example client: `src/libaria/tools/omron_robot_cli.cpp`
