# ROS2_MQTT_PubSub
paho-mqtt + ROS2 Humble

## Prerequisite
Install ROS2 Humble
See [ROS2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)

## Installation
```
mkdir /opt/ros/mqtt_ws/src
cd /opt/ros/mqtt_ws/src
https://github.com/siderdax/ROS2_MQTT_PubSub.git
cd ../
colcon build --symlink-install
```

## Run

### Common
```
source /opt/ros/mqtt_ws/install/setup.bash
```

### Publisher
```
ros2 run ros_mqtt publisher
```

### Subscriber
```
ros2 run ros_mqtt publisher
```

## Configuration

### Remap topic name
See [Remapping](https://design.ros2.org/articles/static_remapping.html)
```
ros2 run ros_mqtt <publisher/subscriber> --ros-args -r <mqtt_pub_message/mqtt_sub_message>:=<new_topic_name>
```

### MQTT Configuration
Setup MQTT is available with ROS2 parameters
1. MQTT hostname
```
ros2 run ros_mqtt <publisher/subscriber> --ros-args -p mqtt_config.host:=<new_address>
```
2. MQTT port
```
ros2 run ros_mqtt <publisher/subscriber> --ros-args -p mqtt_config.port:=<new_tcpip_port>
```
3. MQTT topic
```
ros2 run ros_mqtt <publisher/subscriber> --ros-args -p mqtt_config.topic:=<new_mqtt_topic_name>
```