from setuptools import find_packages, setup
from glob import glob

package_name = 'ros_mqtt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='siderdax',
    maintainer_email='headwaving@gmail.com',
    description='MQTT with ROS2',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = ros_mqtt.mqtt_publisher:main',
            'subscriber = ros_mqtt.mqtt_subscriber:main'
        ],
    },
)
