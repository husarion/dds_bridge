# DDS bridge

This package is intended to allow interoperability across [FastRTPS](https://github.com/eProsima/Fast-RTPS) and [CycloneDDS](https://github.com/eclipse-cyclonedds/cyclonedds) middlewares for ROS2, while FastRTPS is configured in IPv4 network and CycloneDDS is configured in IPv6 network.

The reason to create this package is that [Micro XRCE-DDS Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent), which is based on FastRTPS, is impossible to be configured in IPv6 network. The issue persists as the time of writing this code. 

Above solution should be considered as a temporary workaround.

## Functionality

The `dds_bridge` node subscribes topics published by Micro XRCE-DDS Agent through a direct FastRTPS implementation and publishes those topics on CycloneDDS through rmw_cyclonedds layer.

### Topics published by ROSbot

- `//tf` [geometry_msgs/msg/TransformStamped]
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

Generated code needs to be edited, all the classes must be moved to separate namespace, otherwise they will conflict with CycloneDDS implementation.