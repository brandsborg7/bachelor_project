
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import OutputDevice
from time import sleep

class FanNode(Node):
    def __init__(self):
        super().__init__('fan_node')
        
        # Define the GPIO pins for the two fans
        self.fan_1 = OutputDevice(23, active_high=True, initial_value=False)  # GPIO23 (P4)
        self.fan_2 = OutputDevice(24, active_high=True, initial_value=False)  # GPIO24 (P5)
        self.get_logger().info("Fan_node has been successfully initialized")       

        # Create publishers for the fan statuses
        self.fan_1_status_pub = self.create_publisher(String, 'fan_1_status', 10)
        self.fan_2_status_pub = self.create_publisher(String, 'fan_2_status', 10)

        # Publish initial status of fans
        self.publish_fan_status(1, "OFF")
        self.publish_fan_status(2, "OFF")

    def turn_on_fans(self):
        self.fan_1.on()
        self.fan_2.on()
        self.publish_fan_status(1, "ON")
        self.publish_fan_status(2, "ON")

    def turn_off_fans(self):
        self.fan_1.off()
        self.fan_2.off()
        self.publish_fan_status(1, "OFF")
        self.publish_fan_status(2, "OFF")

    def shutdown(self):
        self.fan_1.off()
        self.fan_2.off()
        self.fan_1.close()
        self.fan_2.close()
        self.get_logger().info("Fans turned OFF and GPIOs closed.")

    def publish_fan_status(self, fan_id, status):
        msg = String()
        if fan_id == 1:
            msg.data = status
            self.fan_1_status_pub.publish(msg)
        elif fan_id == 2:
            msg.data = status
            self.fan_2_status_pub.publish(msg)