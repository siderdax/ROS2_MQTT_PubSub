import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import paho.mqtt.client as mqtt

class MqttPublisher(Node):
    topic = "topic"

    def __init__(self):
        super(MqttPublisher, self).__init__("mqtt_pub")
        self.get_logger().debug(f"topic name: {self.topic}")
        self.mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqttc.on_connect = self.on_connect
        self.mqttc.on_disconnect = self.on_disconnect
        self.mqttc.connect("localhost")
        self.subscribtion = self.create_subscription(
            String, "mqtt_pub_message", self.listener_callback, 10
        )
        self.subscribtion

    def listener_callback(self, msg: String):
        self.mqttc.loop_start()
        self.mqttc.publish(self.topic, msg.data)
        self.mqttc.loop_stop()
        self.get_logger().info('I heard: "%s"' % msg.data)

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            self.get_logger().error(
                f"Failed to connect: {reason_code}. loop_forever() will retry connection"
            )
        else:
            self.get_logger().debug(f"mqttc connected, subscribe {self.topic}")
            client.subscribe(self.topic)

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
