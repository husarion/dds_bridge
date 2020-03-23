# DDS bridge

This package is intended to allow interoperability across [FastRTPS](https://github.com/eProsima/Fast-RTPS) and [CycloneDDS](https://github.com/eclipse-cyclonedds/cyclonedds) middlewares for ROS2, while FastRTPS is configured in IPv4 network and CycloneDDS is configured in IPv6 network.

The reason to create this package is that [Micro XRCE-DDS Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent), which is based on FastRTPS, is impossible to be configured in IPv6 network. The issue persists as the time of writing this code. 

Above solution should be considered as a temporary workaround.

## Functionality

The `dds_bridge` node subscribes topics published by Micro XRCE-DDS Agent through a direct FastRTPS implementation and publishes those topics on CycloneDDS through rmw_cyclonedds layer.

### Topics published by ROSbot

- `/tf` [tf2_msgs/msg/TFMessage]
- `/battery` [sensor_msgs/msg/BatteryState]
- `/odom` [geometry_msgs/msg/PoseStamped]

### Topics subscribed by ROSbot

- `/cmd_vel` [geometry_msgs/msg/Twist]

## Adding new message types

To add a message it is convenient to use [Fast-RTPS Gen](https://github.com/eProsima/Fast-RTPS-Gen) for code generation.
Install the tool according to its documentation.
To generate code for a message:
```
fastrtpsgen PATH_TO_MESSAGE -I MESSAGES_INSTALL_DIRECTORY
```

For `Twist` message from `geometry_msgs` package and default ROS2 Dasing installation:

```
fastrtpsgen /opt/ros/dashing/share/geometry_msgs/msg/Twist.idl -I /opt/ros/dashing/share/
```

Generated code needs some edits.
All the classes must be moved to separate namespace, otherwise they will conflict with CycloneDDS implementation.

Fastrtpsgen creates four files, named according to pattern:
- `<<MessageName>>.cxx`
- `<<MessageName>>.h`
- `<<MessageName>>PubSubTypes.cxx`
- `<<MessageName>>PubSubTypes.h`

For `Twist` these will be: `Twist.cxx`, `Twist.h`, `TwistPubSubTypes.cxx`, `TwistPubSubTypes.h`

Object initializer in `<<MessageName>>PubSubTypes.cxx` defines message type identifier with method `setName()`. The name is constructed as:
```
PACKAGE_NAME::msg::MESSAGE_NAME
```
this needs to be edited:

```
PACKAGE_NAME::msg::dds_::MESSAGE_NAME_
```

Added `dds_::` and underscore `_` folowing the message name.

For every message generated, according entry in `CMakeLists.txt` must be added:

```
add_library(Vector3_lib src/Vector3.cxx)
target_link_libraries(Vector3_lib fastcdr fastrtps)
``` 

and generated library must be linked within `target_link_libraries` directive.

Once the messages are added and edited correctly, project can be built as standard ROS2 package.

## Building the project

Clone the repository to your workspace:

```
git clone https://github.com/husarion/dds_bridge.git
```

Build with colcon:

```
colcon build
```

# Using with ROSbot image

The `dds_bridge` comes preinstalled on image for ROSbot 2.0 along with `CycloneDDS` and other required packages.

Download it from [here](https://husarion-files.s3-eu-west-1.amazonaws.com/ros-dashing-arm-2020-03-22.img.tar.gz) and flash onto a micro SD-card following [system reinstallation manual](https://husarion.com/manuals/rosbot-manual/#rosbot-20).

Before using `Cyclone DDS`, you need to configure it:

- open `cyclonedds.xml` file in home directory:
    ```
    nano ~/cyclonedds.xml
    ```
Find line
    ```
    <NetworkInterfaceAddress></NetworkInterfaceAddress>
    ```
And place IPv6 address of your ROSbot between `NetworkInterfaceAddress` tags.

Then find section `<Peers>` and add `<Peer>` entry for every device you want to use including ROSbot.
Each entry should have `address` property with IPv6 address beteen brackets.
    ```
    <Peers>
        <Peer address="[]"/>
    </Peers>
    ```

You can now start using ROSbot with Cyclone DDS.
- In one terminal start `MicroXRCEAgent`:
    ```
    sudo MicroXRCEAgent serial --dev /dev/ttyS1 -b 500000
    ```
- In second terminal start `dds_bridge`:
    ```
    ros2 run dds_bridge dds_bridge
    ```

## ROS2 API

You can access ROSbot interfaces through IPv4 or IPv6 network interfaces.

### IPv4

ROSbot interfaces are being handled by `MicroXRCEAgent`, this node is directly communicating with CORE2 board on one side and IPv4 network on the other side.

Below topics are available in ROSbot:

| Topic | Message type | Direction | Node |&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Description&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|
| --- | --- | --- | --- | --- |
| `/battery` | `sensor_msgs/msg/BatteryState` | publisher | `MicroXRCEAgent` | Battery voltage |
| `/odom` | `geometry_msgs/msg/PoseStamped` | publisher | `MicroXRCEAgent` | Odometry based on wheel encoders |
| `/tf` | `tf2_msgs/msg/TFMessage` | publisher | `MicroXRCEAgent` | ROSbot position based on wheel encoders |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | subscriber | `MicroXRCEAgent` | Velocity commands |

### IPv6

The interefaces from IPv4 are translated to IPv6 by `dds_bridge`. Thier summary is below:

| Topic | Message type | Direction | Node |&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Description&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|
| --- | --- | --- | --- | --- |
| `/cyclonedds/battery` | `sensor_msgs/msg/BatteryState` | publisher | `dds_bridge` | Battery voltage |
| `/cyclonedds/odom` | `geometry_msgs/msg/PoseStamped` | publisher | `dds_bridge` | Odometry based on wheel encoders |
| `/tf` | `tf2_msgs/msg/TFMessage` | publisher | `dds_bridge` | ROSbot position based on wheel encoders |
| `/cyclonedds/cmd_vel` | `geometry_msgs/msg/Twist` | subscriber | `dds_bridge` | Velocity commands |

### Controlling the ROSbot

To send drive commands you can use `teleop_twist_keyboard`, for issuing commands over IPv4 interface:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

And for issuing commands over IPv6 interface:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard /cmd_vel:=/cyclonedds/cmd_vel
```

# Timestapms

By default CORE is measuring time since reset, thus timestapms are published the same.
If you want to use system time, use `rosbot_time_publisher` node:

```
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run dds_bridge rosbot_time_publisher
```

### External documentation

 - Orbbec Astra camera API is documented in [driver repository](https://github.com/lukaszmitka/ros_astra_camera)

 - Slamtec RpLidar scanner API is documented in [driver repository](https://github.com/lukaszmitka/rplidar_ros)
