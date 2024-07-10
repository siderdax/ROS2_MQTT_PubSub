from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    role = LaunchConfiguration("role")
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    topic = LaunchConfiguration("topic")

    role_larg = DeclareLaunchArgument("role", default_value="subscriber")
    host_larg = DeclareLaunchArgument("host", default_value="localhost")
    port_larg = DeclareLaunchArgument("port", default_value='1883')
    topic_larg = DeclareLaunchArgument("topic", default_value="ros_mqtt_topic")

    mqtt_node = Node(
        package="ros_mqtt",
        executable=role,
        parameters=[
            {
                "mqtt_config": {
                    "host": host,
                    "port": port,
                    "topic": topic,
                }
            },
        ],
    )

    return LaunchDescription([role_larg, host_larg, port_larg, topic_larg, mqtt_node])
