
# ROS2 Communication Libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, RelativeHumidity, FluidPressure
from std_msgs.msg import String, Bool

# Libraries for logging, plotting and file handling 
import time
import logging 
import os
import matplotlib.pyplot as plt
from datetime import datetime

class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')

        # Set up path to the log_file 
        log_dir = '/media/pi/ELLAB/bachelor_prosjekt/ros2_logs/'


        try: 
            os.makedirs(log_dir, exist_ok=True)  # Create the folder if it doesn't already exist
        except Exception as e:
            self.get_logger().error(f"Failed to create log directory: {e}")
            return
          
        log_file = os.path.join(log_dir, f'log_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log')

        # Configure the logger to both log to terminal and to file
        self.logger = logging.getLogger('LoggerNode')
        self.logger.setLevel(logging.INFO)

        # Terminal-handler for logging to terminal
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(message)s')
        console_handler.setFormatter(formatter)
        self.logger.addHandler(console_handler)

        # Filehandler for logging 
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(logging.INFO)
        file_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)

        self.logger.info("Logger Node has been started")
        self.logger.info("----------------------------")

        # Shutdown flag 
        self.shutdown_flag = False

        # Subscriber for shutdown message (from main node)
        self.shutdown_subscriber = self.create_subscription(
            String,
            'shutdown_topic',
            self.shutdown_callback,
            10
        )

        # Buffer lists for saving values 
        self.sensor_1_temp_list = []
        self.sensor_1_humidity_list = []
        self.sensor_1_pressure_list = []

        self.sensor_2_temp_list = []
        self.sensor_2_humidity_list = []
        self.sensor_2_pressure_list = []

        # Subscribers for sensor 1
        self.create_subscription(Temperature, 'sensor1/temperature', self.temp_callback_1, 10)
        self.create_subscription(RelativeHumidity, 'sensor1/humidity', self.humidity_callback_1, 10)
        self.create_subscription(FluidPressure, 'sensor1/pressure', self.pressure_callback_1, 10)

        # Subscribers for sensor 2
        self.create_subscription(Temperature, 'sensor2/temperature', self.temp_callback_2, 10)
        self.create_subscription(RelativeHumidity, 'sensor2/humidity', self.humidity_callback_2, 10)
        self.create_subscription(FluidPressure, 'sensor2/pressure', self.pressure_callback_2, 10)

        # Subscriber for lift status
        self.create_subscription(String, 'lift_status', self.lift_status_callback, 10)

        # [2 x RC-SERVO] Subscriber for RC-servo (lid) status
        self.create_subscription(String, 'rc_servo_1_status', self.rc_servo_1_status_callback, 10)
        self.create_subscription(String, 'rc_servo_2_status', self.rc_servo_2_status_callback, 10)

        # [2 x FAN] Subscriber for fan status
        self.create_subscription(String, 'fan_1_status', self.fan_1_status_callback, 10)
        self.create_subscription(String, 'fan_2_status', self.fan_2_status_callback, 10)   

        # [2 x camera] subscriber for camera status
        self.create_subscription(Bool, 'camera0_status', self.camera_1_status_callback, 10)
        self.create_subscription(Bool, 'camera2_status', self.camera_2_status_callback, 10)

        # Initialize flags to check if the statuses have been logged initially
        self.logged_camera_1_status = False
        self.logged_camera_2_status = False

        # For storing sensor data for later printing
        self.sensor_1_data = []
        self.sensor_2_data = []

        # Saving timestamps for the plot
        self.timestamps = []

        # For storing lift & servo status & fan status
        self.lift_status = ""
        self.rc_servo_1_status = ""
        self.rc_servo_2_status = ""
        self.fan_1_status = ""
        self.fan_2_status = ""

        # Storing camera status
        self.camera_1_status = False
        self.camera_2_status = False

        # Connection flags
        self.sensor_1_connected = False
        self.sensor_2_connected = False
        self.lift_connected = False
        self.rc_servo_1_connected = False
        self.rc_servo_2_connected = False
        self.fan_1_connected = False
        self.fan_2_connected = False

        # Store timestamps to check if messages are not coming in within a certain time
        self.last_sensor_1_msg_time = time.time()
        self.last_sensor_2_msg_time = time.time()
        self.last_lift_status_time = time.time()
        self.last_rc_servo_1_status_time = time.time()
        self.last_rc_servo_2_status_time = time.time()
        self.last_fan_1_status_time = time.time()
        self.last_fan_2_status_time = time.time()

        # Timeout threshold for checking connections
        self.timeout_threshold = 2  # seconds

    def update_sensor_connection(self, sensor_id):
        if sensor_id == 1:
            self.sensor_1_connected = True
            self.last_sensor_1_msg_time = time.time() # Reset the timer

        elif sensor_id == 2:
            self.sensor_2_connected = True
            self.last_sensor_2_msg_time = time.time()

    def temp_callback_1(self, msg):
        self.sensor_1_temp_list.append(msg.temperature)
        self.sensor_1_data.append(f"Sensor 1 - Temperature: {msg.temperature:.2f} °C")
        self.update_sensor_connection(1)
        self.print_sensor_data()

    def humidity_callback_1(self, msg):
        self.sensor_1_humidity_list.append(msg.relative_humidity)
        self.sensor_1_data.append(f"Sensor 1 - Humidity: {msg.relative_humidity:.2f} %")
        self.update_sensor_connection(1)
        self.print_sensor_data()

    def pressure_callback_1(self, msg):
        self.sensor_1_pressure_list.append(msg.fluid_pressure)
        self.sensor_1_data.append(f"Sensor 1 - Pressure: {msg.fluid_pressure:.2f} hPa")
        self.update_sensor_connection(1)
        self.print_sensor_data()

    def temp_callback_2(self, msg):
        self.sensor_2_temp_list.append(msg.temperature)
        self.sensor_2_data.append(f"Sensor 2 - Temperature: {msg.temperature:.2f} °C")
        self.update_sensor_connection(2)
        self.print_sensor_data()

    def humidity_callback_2(self, msg):
        self.sensor_2_humidity_list.append(msg.relative_humidity)
        self.sensor_2_data.append(f"Sensor 2 - Humidity: {msg.relative_humidity:.2f} %")
        self.update_sensor_connection(2)
        self.print_sensor_data()

    def pressure_callback_2(self, msg):
        self.sensor_2_pressure_list.append(msg.fluid_pressure)
        self.sensor_2_data.append(f"Sensor 2 - Pressure: {msg.fluid_pressure:.2f} hPa")
        self.update_sensor_connection(2)
        self.print_sensor_data()

    def lift_status_callback(self, msg):
        self.lift_status = msg.data
        self.lift_connected = True
        self.last_lift_status_time = time.time()  # Reset the timer
        self.print_lift_status()

    def rc_servo_1_status_callback(self, msg):
        if msg.data != self.rc_servo_1_status:
            self.rc_servo_1_status = msg.data
            self.rc_servo_1_connected = True
            self.print_rc_servo_status(servo_id=1)

    def rc_servo_2_status_callback(self, msg):
        if msg.data != self.rc_servo_2_status:
            self.rc_servo_2_status = msg.data
            self.rc_servo_2_connected = True
            self.print_rc_servo_status(servo_id=2)

    def fan_1_status_callback(self, msg):
        if msg.data != self.fan_1_status:
            self.fan_1_status = msg.data
            self.fan_1_connected = True
            self.print_fan_status(fan_id=1)

    def fan_2_status_callback(self, msg):
        if msg.data != self.fan_2_status:
            self.fan_2_status = msg.data
            self.fan_2_connected = True
            self.print_fan_status(fan_id=2)      

    def camera_1_status_callback(self, msg):
        # If the status is different from the previous status or if it's the first time logging
        if msg.data != self.camera_1_status or not self.logged_camera_1_status:
            self.camera_1_status = msg.data
            self.print_camera_status(camera_id=0)
            self.logged_camera_1_status = True  # Set the flag to True after first log

    def camera_2_status_callback(self, msg):
        # If the status is different from the previous status or if it's the first time logging
        if msg.data != self.camera_2_status or not self.logged_camera_2_status:
            self.camera_2_status = msg.data
            self.print_camera_status(camera_id=1)
            self.logged_camera_2_status = True  # Set the flag to True after first log

    def shutdown_callback(self, msg: String):
        self.logger.info(f"Received shutdown message: {msg.data}")
        
        if msg.data == "shutdown":
            self.logger.info("Logger node shutting down...")
            self.save_plots()
            self.shutdown_flag = True

    def print_camera_status(self, camera_id):
        status = 'ON' if (self.camera_1_status if camera_id == 0 else self.camera_2_status) else 'OFF'
        self.logger.info(f"[INFO] - Camera {camera_id + 1} Status: {status}")
        self.logger.info("----------------------------")

    def print_sensor_data(self):
        if len(self.sensor_1_data) == 3:
            self.logger.info("[INFO] - Sensor 1 Data (Inside Bucket):")
            for data in self.sensor_1_data:
                self.logger.info(data)
            self.logger.info("----------------------------")
            self.sensor_1_data = []  # Reset data for Sensor 1

        if len(self.sensor_2_data) == 3:
            self.logger.info("[INFO] - Sensor 2 Data (Ambient):")
            for data in self.sensor_2_data:
                self.logger.info(data)
            self.logger.info("----------------------------")
            self.sensor_2_data = []  # Reset data for Sensor 2

    def print_lift_status(self):
        if self.lift_status:
            self.logger.info("[INFO] - Lift Status:")
            self.logger.info(self.lift_status)
            self.logger.info("----------------------------")

    def print_rc_servo_status(self, servo_id):
        if servo_id == 1 and self.rc_servo_1_status:
            self.logger.info("[INFO] - RC-Servo 1 Status:")
            self.logger.info(self.rc_servo_1_status)
            self.logger.info("----------------------------")
        elif servo_id == 2 and self.rc_servo_2_status:
            self.logger.info("[INFO] - RC-Servo 2 Status:")
            self.logger.info(self.rc_servo_2_status)
            self.logger.info("----------------------------")

    def print_fan_status(self, fan_id):
        if fan_id == 1 and self.fan_1_status:
            self.logger.info("[INFO] - Fan 1 Status:")
            self.logger.info(self.fan_1_status)
            self.logger.info("----------------------------")
        elif fan_id == 2 and self.fan_2_status:
            self.logger.info("[INFO] - Fan 2 Status:")
            self.logger.info(self.fan_2_status)
            self.logger.info("----------------------------")    

    def save_plots(self):

        # Check if there are any values to plot
        if not (self.sensor_1_temp_list or self.sensor_2_temp_list):
            self.logger.warning("No sensors connected & hence no data available to plot.")
            return

        save_dir = '/media/pi/ELLAB/bachelor_prosjekt/sensor_plots/'


        try: 
            os.makedirs(save_dir, exist_ok=True)
        except Exception as e:
            self.logger.error(f"Failed to create plot directory: {e}")
            return

        # Date & time for unique filename
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        saved_files = []

        # Prepare data and plot details in a list of dicts
        plots = [
            {
                'data': [self.sensor_1_temp_list, self.sensor_2_temp_list],
                'labels': ['Sensor 1 (Inside Bucket)- Temperature (°C)', 'Sensor 2 (Ambient) - Temperature (°C)'],
                'colors': ['red', 'orange'],
                'title': 'Temperature Comparison',
                'ylabel': 'Temperature (°C)',
                'filename': f"temperature_comparison_{timestamp}.png"
            },
            {
                'data': [self.sensor_1_humidity_list, self.sensor_2_humidity_list],
                'labels': ['Sensor 1 (Inside Bucket) - Humidity (%)', 'Sensor 2 (Ambient) - Humidity (%)'],
                'colors': ['blue', 'cyan'],
                'title': 'Humidity Comparison',
                'ylabel': 'Humidity (%)',
                'filename': f"humidity_comparison_{timestamp}.png"
            },
            {
                'data': [self.sensor_1_pressure_list, self.sensor_2_pressure_list],
                'labels': ['Sensor 1 (Inside Bucket) - Pressure (hPa)', 'Sensor 2 (Ambient) - Pressure (hPa)'],
                'colors': ['green', 'lime'],
                'title': 'Pressure Comparison',
                'ylabel': 'Pressure (hPa)',
                'filename': f"pressure_comparison_{timestamp}.png"
            },
        ]

        # Create plots for each set of data (temperature, humidity, pressure)
        for plot in plots:
            plt.figure(figsize=(10, 6))
            for data, label, color in zip(plot['data'], plot['labels'], plot['colors']):
                plt.plot(data, label=label, color=color)
            plt.title(plot['title'])
            plt.xlabel('Sample')
            plt.ylabel(plot['ylabel'])
            plt.legend()
            plt.grid(True)
            plt.savefig(os.path.join(save_dir, plot['filename']))
            plt.close()
            saved_files.append(plot['filename'])
        
        self.logger.info(f"Plots saved: {', '.join(saved_files)}")

def main(args=None):
    rclpy.init(args=args)
    logger_node = LoggerNode()

    while rclpy.ok() and not logger_node.shutdown_flag:
        rclpy.spin_once(logger_node)  
        time.sleep(0.1)  # Sleep to avoid busy-waiting

    # Cancelling node & ROS when while-loop is done
    logger_node.get_logger().info("Shutting down Logger Node...")
    logger_node.destroy_node()  # Shutting down the logger_node
    rclpy.shutdown()  # Cleaning up and stopping ROS2


if __name__ == '__main__':
    main()
