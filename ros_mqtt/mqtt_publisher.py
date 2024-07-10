from typing import List
import rclpy
from rclpy.node import Node
from rclpy import Parameter
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import String

import paho.mqtt.client as mqtt


class MqttPublisher(Node):
    mqtt_config = {"host": "localhost", "port": 1883}
    topic = "ros_mqtt"

    def __init__(self):
        super(MqttPublisher, self).__init__("mqtt_pub")
        self.mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqttc.on_connect = self.on_connect
        self.mqttc.on_disconnect = self.on_disconnect
        self.add_on_set_parameters_callback(self.on_set_parameters)
        self.declare_parameter("mqtt_config.host", "localhost"),
        self.declare_parameter("mqtt_config.port", 1883),
        self.declare_parameter("mqtt_config.topic", "ros_mqtt"),
        # self.declare_parameters(
        #     "mqtt_config",
        #     [
        #         ("host", "localhost"),
        #         ("port", 1883),
        #         ("topic", "ros_mqtt"),
        #     ],
        # )
        host = self.mqtt_config["host"]
        port = self.mqtt_config["port"]
        self.get_logger().info(f"connect mqttc {host}:{port}")
        self.mqttc.connect(host, port)
        self.subscribtion = self.create_subscription(
            String, "mqtt_pub_message", self.listener_callback, 10
        )
        self.subscribtion

    def on_set_parameters(self, params: List[Parameter]):
        for param in params:
            config_name = param.name.replace("mqtt_config.", "")
            if (
                config_name in self.mqtt_config
                and self.mqtt_config[config_name] != param.value
            ):
                self.get_logger().info(
                    f"config {config_name} is changed from {self.mqtt_config[config_name]} to {param.value}"
                )
                self.mqtt_config[config_name] = param.value

            if config_name == "topic" and self.topic != param.value:
                self.get_logger().info(
                    f"topic is changed from {self.topic} to {param.value}"
                )
                self.topic = param.value

        return SetParametersResult(successful=True)

    def listener_callback(self, msg: String):
        try:
            if (
                self.mqttc.host != self.mqtt_config["host"]
                or self.mqttc.port != self.mqtt_config["port"]
            ):
                self.mqttc.disconnect()
                self.mqttc.connect(self.mqtt_config["host"], self.mqtt_config["port"])
            elif self.mqttc.is_connected == False:
                self.mqttc.connect(self.mqtt_config["host"], self.mqtt_config["port"])

            self.mqttc.loop_start()
            self.mqttc.publish(self.topic, msg.data)
            self.mqttc.loop_stop()
            self.get_logger().info('I heard: "%s"' % msg.data)
        except:
            pass

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            self.get_logger().error(
                f"Failed to connect: {reason_code}. loop_forever() will retry connection"
            )
        else:
            self.get_logger().debug(
                f"mqttc connected {self.mqttc.host}:{self.mqttc.port}"
            )

    def on_disconnect(self, client, userdata, flags, reason_code, properties):
        self.get_logger().debug("disconnect mqttc")


def main(args=None):
    rclpy.init(args=args)
    mqtt_pub = MqttPublisher()
    rclpy.spin(mqtt_pub)
    mqtt_pub.mqttc.disconnect()
    mqtt_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
