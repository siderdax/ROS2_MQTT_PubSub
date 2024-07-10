from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    node_name = LaunchConfiguration("node_name")
    topic_name = LaunchConfiguration("topic_name")
    role = LaunchConfiguration("role")
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    topic = LaunchConfiguration("topic")

    node_name_larg = DeclareLaunchArgument("node_name", default_value="ros_mqtt")
    topic_name_larg = DeclareLaunchArgument("topic_name", default_value="mqtt_message")
    role_larg = DeclareLaunchArgument("role", default_value="subscriber")
    host_larg = DeclareLaunchArgument("host", default_value="localhost")
    port_larg = DeclareLaunchArgument("port", default_value="1883")
    topic_larg = DeclareLaunchArgument("topic", default_value="/ros_mqtt_topic")

    mqtt_node = Node(
        package="ros_mqtt",
        name=node_name,
        remappings=[
            ('/mqtt_pub_message', topic_name),
            ('/mqtt_sub_message', topic_name),
        ],
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

    return LaunchDescription(
        [
            node_name_larg,
            topic_name_larg,
            role_larg,
            host_larg,
            port_larg,
            topic_larg,
            mqtt_node,
        ]
    )
