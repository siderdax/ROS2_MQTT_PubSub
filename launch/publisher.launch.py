from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [FindPackageShare("ros_mqtt"), "launch", "mqtt.launch.py"]
                        )
                    ],
                ),
                launch_arguments={
                    "role": "publisher",
                    "node_name": "mqtt_pub",
                    "topic_name": "mqtt_pub_message",
                }.items(),
            ),
        ]
    )
