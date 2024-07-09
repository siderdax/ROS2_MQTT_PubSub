from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros_mqtt",
                executable="subscriber",
                parameters=[
                    {
                        "mqtt_config": {
                            "host": "127.0.0.1",
                            "port": 1883,
                            "topic": "custom_topic",
                        }
                    },
                ],
                output="screen",
                emulate_tty=True,
            )
        ]
    )
