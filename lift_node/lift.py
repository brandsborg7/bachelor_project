# Libraries for ROS2 Communication
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Library for GPIO control
from gpiozero import OutputDevice, PWMOutputDevice

# Define GPIO pins
IN1_PIN = 20  # GPIO pin for IN1 (M1)
IN2_PIN = 21  # GPIO pin for IN2 (M2)
PWM_PIN = 26  # GPIO pin for PWM (PWMA)

class LiftNode(Node):
    def __init__(self):
        super().__init__('lift_node')

        # Try to initialize GPIO motor control
        try:
            self.in1 = OutputDevice(IN1_PIN)
            self.in2 = OutputDevice(IN2_PIN)
            self.pwm_motor = PWMOutputDevice(PWM_PIN)
            self.pwm_motor.frequency = 1000
            self.pwm_motor.value = 0.99  # 99% duty cycle
            self.last_status_message = None
            self.get_logger().info("Lift Node has been initialized successfully")
        except Exception as e:
            self.in1 = self.in2 = self.pwm_motor = None
            self.get_logger().error(f"Failed to initialize the lift: {e}")

        # ROS 2 Publisher and Subscriber
        self.lift_status_publisher = self.create_publisher(String, 'lift_status', 10)
        self.cmd_subscriber = self.create_subscription(String, 'lift_cmd', self.lift_command_callback, 10)

        # Publish initial status of the lift
        if self.check_lift_status():
            self.publish_status("start position")
        else:
            self.publish_status("Lift is not connected") 

        # Timer for lift test
        self.test_timer = None

    def check_lift_status(self):
        return self.in1 is not None and self.in2 is not None and self.pwm_motor is not None

    def lift_command_callback(self, msg):
        command = msg.data
        if not self.check_lift_status():
            self.publish_status("Lift is not connected or malfunctioning")
            return

        if command == "lift_up":
            self.lift_up()
        elif command == "lift_down":
            self.lift_down()
        elif command == "lift_stop":
            self.lift_stop()
        elif command == "lift_test":
            self.start_lift_test_sequence()
        else:
            self.publish_status(f"Unknown command: {command}")

    # Lift (Motor) Control Functions 
    def lift_down(self, duration=None):
        self.in1.off()
        self.in2.on()
        self.publish_status("Lift is set to move downwards")
        
        if duration:
            self.reset_timer()
            self.test_timer = self.create_timer(duration, self.lift_stop)  # Stop after specified duration

    def lift_up(self, duration=None):
        self.in1.on()
        self.in2.off()
        self.publish_status("Lift is set to move upwards")

        if duration:
            self.reset_timer()
            self.test_timer = self.create_timer(duration, self.lift_stop) # Stop after specified duration

    def lift_stop(self):
        self.in1.off()
        self.in2.off()
        self.publish_status("Lift stopped")
        self.test_timer = None

    def reset_timer(self):
        if self.test_timer is not None:
            self.test_timer.cancel()
            self.test_timer = None

    def start_lift_test_sequence(self):
        self.test_phase = 0
        self.publish_status("Starting lift test")
        self.run_test_step()

    def run_test_step(self):
        self.reset_timer()

        if self.test_phase == 0:
            self.lift_down()
            self.test_timer = self.create_timer(40.0, self.run_test_step)
            self.test_phase = 1

        elif self.test_phase == 1:
            self.lift_stop()
            self.test_timer = self.create_timer(2.0, self.run_test_step)
            self.test_phase = 2

        elif self.test_phase == 2:
            self.lift_up()
            self.test_timer = self.create_timer(40.0, self.run_test_step)
            self.test_phase = 3

        elif self.test_phase == 3:
            self.lift_stop()
            self.publish_status("Lift test completed")
            self.test_phase = None

    # Publish status messages
    def publish_status(self, status_message):

        if status_message != self.last_status_message:
            msg = String()
            msg.data = status_message
            self.lift_status_publisher.publish(msg)
            self.get_logger().info(status_message)
            self.last_status_message = status_message


