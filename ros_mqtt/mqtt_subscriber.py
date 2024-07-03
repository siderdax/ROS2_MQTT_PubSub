from typing import List
import rclpy
from rclpy.node import Node
from rclpy import Parameter
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import String

import paho.mqtt.client as mqtt
import threading


class MqttSubscriber(Node):
    mqtt_config = {"host": "localhost", "port": 1883, "topic": "ros_mqtt"}

    def __init__(self):
        super(MqttSubscriber, self).__init__("mqtt_sub")
        self.declare_parameters(
            "mqtt_config",
            [
                ("host", "localhost"),
                ("port", 1883),
                ("topic", "ros_mqtt"),
            ],
        )
        self.add_on_set_parameters_callback(self.on_set_parameters)
        self.mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqttc.on_connect = self.on_connect
        self.mqttc.on_disconnect = self.on_disconnect
        self.mqttc.on_message = self.on_message
        self.mqttc.on_subscribe = self.on_subscribe
        self.mqttc.on_unsubscribe = self.on_unsubscribe
        self.mqttc_thread = threading.Thread(target=self.mqtt_loop)
        self.mqttc_thread.start()
        self.publisher = self.create_publisher(String, "mqtt_sub_message", 10)

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
                self.mqttc.disconnect()
                self.mqttc_thread = threading.Thread(target=self.mqtt_loop)
                self.mqttc_thread.start()

        return SetParametersResult(successful=True)

    def on_subscribe(self, client, userdata, mid, reason_code_list, properties):
        if reason_code_list[0].is_failure:
            self.get_logger().error(
                f"Broker rejected you subscription: {reason_code_list[0]}"
            )
        else:
            self.get_logger().info(
                f"Broker granted the following QoS: {reason_code_list[0].value}"
            )

    def on_unsubscribe(self, client, userdata, mid, reason_code_list, properties):
        if len(reason_code_list) == 0 or not reason_code_list[0].is_failure:
            self.get_logger().debug(
                "unsubscribe succeeded (if SUBACK is received in MQTTv3 it success)"
            )
        else:
            self.get_logger().warn(
                f"Broker replied with failure: {reason_code_list[0]}"
            )
        client.disconnect()

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            self.get_logger().error(
                f"Failed to connect: {reason_code}. loop_forever() will retry connection"
            )
        else:
            topic = self.mqtt_config["topic"]
            self.get_logger().debug(f"mqttc connected, subscribe {topic}")
            client.subscribe(topic)

    def on_disconnect(self, client, userdata, flags, reason_code, properties):
        self.get_logger().debug("disconnect mqttc")

    def on_message(self, client, userdata, msg):
        self.get_logger().info(msg.topic + " " + str(msg.payload))
        pub_msg = String()
        pub_msg.data = msg.payload.decode("utf-8").replace("\r\n", "\n")
        self.publisher.publish(pub_msg)

    def mqtt_loop(self):
        self.get_logger().debug("connect mqttc")
        self.mqttc.connect(self.mqtt_config["host"], self.mqtt_config["port"])
        self.mqttc.loop_forever()


def main(args=None):
    rclpy.init(args=args)
    mqtt_sub = MqttSubscriber()
    rclpy.spin(mqtt_sub)
    mqtt_sub.mqttc.disconnect()
    mqtt_sub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
