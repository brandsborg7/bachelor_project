import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import time
import threading

# Importing necessary modules for the main node
from lift_node.lift import LiftNode
from sensor_node.sensor import SensorPublisher
from rc_servo_node.rc_servo import RCServoNode
from fan_node.fan import FanNode
from camera_node.camera import CameraNode

class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')

        # Log the node startup using ROS 2 logger
        self.get_logger().info("Main Node has been started")
        self.running = True
        self.shutdown_everything = False

        # Initialize the nodes (with error handling)
        self.lift_node = self.safe_init(LiftNode)
        self.sensor_node = self.safe_init(SensorPublisher)  
        self.rc_servo_node = self.safe_init(RCServoNode)
        self.fan_node = self.safe_init(FanNode)
        self.camera1_node = self.safe_init(CameraNode, 0)
        self.camera2_node = self.safe_init(CameraNode, 2)
        
        # Publisher for the lift command 
        self.lift_cmd_publisher = self.create_publisher(String, 'lift_cmd', 10)

        # Publisher for shutdown message to logger node
        self.shutdown_pub = self.create_publisher(String, 'shutdown_topic', 10)

        # Lift status subscriber (for sequence control)
        self.lift_status_sub = self.create_subscription(String, 'lift_status', self.lift_status_callback, 10)
        self.lift_in_stopped_condition = True
        self.lift_sequence_phase = None
        self.sequence_active = False

    # Function to safely initialize nodes (w/its arguments and keyword-arguments) and handle exceptions
    def safe_init(self, class_name, *args, **kwargs):
        try:
            return class_name(*args, **kwargs)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize {class_name.__name__}: {e}")
            return None

    # Function to send shutdown message to the logger node
    def shutdown_logger(self):
        msg = String()
        msg.data = 'shutdown'
        self.shutdown_pub.publish(msg)
        self.get_logger().info('Sendt shutdown message to logger_node.')

    def send_lift_command(self, command: str):
        try:
            msg = String()
            msg.data = command
            self.lift_cmd_publisher.publish(msg)
            self.get_logger().info(f"Sent command: {command}")
        except Exception as e:
            self.get_logger().error(f"Failed to send lift command: {e}, command: {command}")

    def shutdown_nodes(self):

        # Stop the command loop
        self.running = False

        nodes_to_shutdown = [
            self.lift_node,
            self.sensor_node,
            self.rc_servo_node,
            self.fan_node,
            self.camera1_node,
            self.camera2_node,
        ]

        for node in nodes_to_shutdown:
            if node:
                try:
                    node_name = node.get_name()
                    node.destroy_node()
                    self.get_logger().info(f"Node {node_name} has been destroyed.")
                except Exception as e:
                    self.get_logger().error(f"Failed to destroy node: {e}")
            else:
                self.get_logger().warn("Node is None, skipping destruction.")
        
        self.get_logger().info("Main node shutting down.")

    # Function to start multiple threads simultaneously (just starting them)
    def start_threads(self, funcs):
        threads = []
        for f in funcs:
            thread = threading.Thread(target=f)
            thread.start()
            threads.append(thread)
        return threads
    
    def run_thread_and_wait(self, funcs):
        threads = self.start_threads(funcs)
        for thread in threads:
            thread.join()

    def lift_status_callback(self, msg):
        status = msg.data
        self.get_logger().info(f"Received lift status: {status}")

        # Only handle lift status when in full sequence
        if not self.sequence_active:
            return

        if status == "Lift stopped":

            # Step 5: Close the LIDs (servos), turn off fans 
            if self.sequence_phase == 1:
                # Lift down just finished

                if self.rc_servo_node:
                    self.run_thread_and_wait([
                    lambda: self.rc_servo_node.servo_1.process_command("close"),
                    lambda: self.rc_servo_node.servo_2.process_command("close"),
            ])

                self.get_logger().info("Lift down completed. Closing servos.")
                
                self.get_logger().info("Servos closed. Turning off fans.")
                self.fan_node.turn_off_fans()

                if self.lift_node:
                    self.lift_node.lift_up(duration=40)
                self.sequence_phase = 2

            # Step 6 (final): Lift up, stop the sequence
            elif self.sequence_phase == 2:
                # Lift up just finished
                self.get_logger().info("Lift up completed. Turning off cameras")
                
                # Stop the cameras 
                for thread in self.camera_threads:
                    thread.join()
                self.lift_in_stopped_condition = True
                self.sequence_phase = None
                self.sequence_active = False 
                self.get_logger().info("Full sequence completed.")

    def full_sequence(self): 
        self.get_logger().info("Starting full sequence...")

        if not all([self.lift_node, self.fan_node, self.rc_servo_node]):
            self.get_logger().error("❌ One or more required nodes are not initialized. Aborting sequence.")
            return

        if not self.lift_in_stopped_condition:
            self.get_logger().warn("⚠️ Lift is currently moving. Wait for it to stop before running sequence.")
            return
        
        self.sequence_active = True  # Sequence is now active
        self.lift_in_stopped_condition = False

        try: 

            # Step 1: Start camera recordings 
            self.get_logger().info("🎥 Starting camera recordings (100s)...")
            self.camera_threads = self.start_threads([
                lambda: self.camera1_node.create_video(100),
                lambda: self.camera2_node.create_video(100),
            ])

            # STEP2: Open the LIDs w/servos 
            self.get_logger().info("🔓 Opening servos...")
            self.run_thread_and_wait([
                    lambda: self.rc_servo_node.servo_1.process_command("open"),
                    lambda: self.rc_servo_node.servo_2.process_command("open"),
                    ])
            self.get_logger().info("✅ Servos opened.")

            # Step 3: Lift down 
            self.get_logger().info("⬇️ Lowering lift and turning on fans...")
            self.lift_node.lift_down(duration=40)
            self.sequence_phase = 1  # So that it will go to next phase when lift stops 
            
            # Step 4: Turn on fans
            self.fan_node.turn_on_fans() # Turn on the fans 
            self.get_logger().info("💨 Fans turned on. Waiting for lift to stop...")
        except Exception as e:
            self.get_logger().error(f"❌ Error during full sequence: {e}")
            self.sequence_active = False
            self.sequence_phase = None
 
    def exit(self):
        self.get_logger().info("Exiting main node.")

        # Turn off fans and servos
        if self.fan_node: self.fan_node.shutdown()
        if self.rc_servo_node: self.rc_servo_node.shutdown()

        self.running = False
        self.shutdown_everything = True
        self.shutdown_logger()
        rclpy.shutdown()
  
    # Main loop for command input (sending commands to the lift through the main-terminal)
    def run_command_loop(self):
        while rclpy.ok() and self.running:
            try:
                command = input("Enter command (e.g. lift_up, lift_down, lift_stop, " \
                "lift_test, fans_on, rc_test, rc_open, rc_close, cam_test, run_all, exit): ")

                if command == "lift_up": self.send_lift_command("lift_up")
                elif command == "lift_down": self.send_lift_command("lift_down")
                elif command == "lift_stop":
                    self.send_lift_command("lift_stop")
                    self.lift_in_stopped_condition = True
                elif command == "lift_test": self.send_lift_command("lift_test")
                elif command == "fans_on": self.fan_node.turn_on_fans()
                elif command == "rc_test":
                    # Test both servos by opening and closing them (at the same time using threads)
                    self.run_thread_and_wait([
                        lambda: self.rc_servo_node.servo_1.process_command("open"),
                        lambda: self.rc_servo_node.servo_2.process_command("open"),
                    ])

                    # Wait for 1 second before closing the servos 
                    time.sleep(1)  
                    self.run_thread_and_wait([
                        lambda: self.rc_servo_node.servo_1.process_command("close"),
                        lambda: self.rc_servo_node.servo_2.process_command("close"),
                    ])
        
                elif command == "rc_open":
                    self.run_thread_and_wait([
                        lambda: self.rc_servo_node.servo_1.process_command("open"),
                        lambda: self.rc_servo_node.servo_2.process_command("open"),
                    ])
                elif command == "rc_close":
                    self.run_thread_and_wait([
                        lambda: self.rc_servo_node.servo_1.process_command("close"),
                        lambda: self.rc_servo_node.servo_2.process_command("close"),
                    ])
                elif command == "cam_test":
                    self.get_logger().info("Starting camera test (5 seconds)...")

                    self.run_thread_and_wait([
                        lambda: self.camera1_node.create_video(5),
                        lambda: self.camera2_node.create_video(5),
                    ])

                    self.get_logger().info("Camera test completed.")
                elif command == "run_all": self.full_sequence()
                elif command == "exit": 
                    self.exit()
                    break
                else:
                    self.get_logger().info(f"Unknown command: {command}")
                
                print("Command completed. Enter a new command:\n")

            except KeyboardInterrupt:
                        self.get_logger().info("Main node interrupted by keyboard")
                        self.running = False
                        break

def main(args=None):

    main_node = None    # Initialize main_node to None to avoid reference before assignment
    executor = None     # Initialize executor to None to avoid reference before assignment
    command_thread = None  # Initialize command_thread to None to avoid reference before assignment

    try:
        rclpy.init(args=args)  # Initialize the ROS 2 context
        main_node = MainNode()

        # Bruk MultiThreadedExecutor for å spinne alle nodene
        executor = MultiThreadedExecutor()
        executor.add_node(main_node)  # Add the main node to the executor

        # Add all the child nodes to the executor (if they are not None)
        for node in [main_node.lift_node, main_node.rc_servo_node, main_node.fan_node, 
                     main_node.camera1_node, main_node.camera2_node]:
            if node:
                executor.add_node(node)

        # Add the sensor node only if at least one sensor is available
        if main_node.sensor_node and main_node.sensor_node.sensor1 and main_node.sensor_node.sensor2:
            executor.add_node(main_node.sensor_node)
        else:
            main_node.get_logger().warn("One or both sensors are not available, skipping addition to executor.")
        
        # Start the command loop in a separate thread (to avoid blocking the executor)
        command_thread = threading.Thread(target=main_node.run_command_loop, daemon=True)
        command_thread.start()

        # Spin all the nodes in the executor
        executor.spin()

    except KeyboardInterrupt:
        if main_node:
            main_node.get_logger().info("Main node interrupted by user")

    except Exception as e:
        if main_node:
            main_node.get_logger().error(f"Unexpected error occurred in main node: {e}")

    finally:

        if main_node and main_node.shutdown_everything:

            if executor:
                executor.shutdown()  # Shutdown the executor properly

            if main_node:
                main_node.shutdown_nodes()  # Shutdown all nodes
                main_node.destroy_node()

            # Wait for the command thread to finish        
            if command_thread and command_thread.is_alive():
                try: 
                    command_thread.join()
                except RuntimeError:
                    if main_node:
                        main_node.get_logger().error("[RuntimeError]: Error while waiting for command thread to finish.")

            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
