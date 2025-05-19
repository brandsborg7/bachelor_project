# ROS2 Libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# GPIO control
from gpiozero import Servo
from time import sleep

class RCServo:
    def __init__(self, pin, publisher, logger, name="rc_servo",
                 open_value=-0.5, close_value=0.2, release_after_move=True):
        self.servo = Servo(pin, min_pulse_width=900/1_000_000, max_pulse_width=2100/1_000_000)
        self.publisher = publisher
        self.logger = logger
        self.name = name
        self.open_value = open_value
        self.close_value = close_value
        self.release_after_move = release_after_move

        self.servo.value = None  # Avoid holding on boot
        self.publish_status("closed")

    def process_command(self, command):
        if command == "open":
            self.move_to(self.open_value, "open")
        elif command == "close":
            self.move_to(self.close_value, "closed")
        else:
            self.logger.warning(f"[{self.name}] Unknown command: {command}")

    def move_to(self, target_position, status_label):
        try:
            self.logger.info(f"[{self.name}] Moving to {status_label} (value={target_position})")
            self.servo.value = target_position
            sleep(0.25)  
            if self.release_after_move:
                self.servo.value = None  # Release control after move
            self.publish_status(status_label)
        except Exception as e:
            self.logger.error(f"[{self.name}] Error moving servo: {e}")

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.publisher.publish(msg)


class RCServoNode(Node):
    def __init__(self):
        super().__init__('rc_servo_node')
        self.get_logger().info("RC Servo Node started")

        # Publishers for each servo's status
        self.servo_1_pub = self.create_publisher(String, 'rc_servo_1_status', 10)
        self.servo_2_pub = self.create_publisher(String, 'rc_servo_2_status', 10)

        # RCServo instances with specific open/close values per servo
        self.servo_1 = RCServo(18, self.servo_1_pub, self.get_logger(), name="rc_servo_1",
                               open_value=-0.5, close_value=0.2, release_after_move=True)
        self.servo_2 = RCServo(25, self.servo_2_pub, self.get_logger(), name="rc_servo_2",
                               open_value=0.15, close_value=0.5, release_after_move=True)

        # Subscribers for each servo commands
        self.create_subscription(String, 'rc_servo_1_command', self.rc_servo_1_callback, 10)
        self.create_subscription(String, 'rc_servo_2_command', self.rc_servo_2_callback, 10)

    def rc_servo_1_callback(self, msg):
        self.servo_1.process_command(msg.data.lower())

    def rc_servo_2_callback(self, msg):
        self.servo_2.process_command(msg.data.lower())

    def shutdown(self):
        self.get_logger().info("Closing the RC-Servos...")

        # Move servos to "closed" position and release control
        self.servo_1.process_command("close")
        self.servo_2.process_command("close")

        self.servo_1.servo.value = None
        self.servo_2.servo.value = None

        try:
            self.servo_1.servo.close()
            self.servo_2.servo.close()
        except Exception as e:
            self.get_logger().warn(f"Error while closing servo GPIO: {e}")
