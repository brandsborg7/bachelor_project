# Libraries for ROS2 Communication
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, RelativeHumidity, FluidPressure
from std_msgs.msg import String, Bool

# Libraries for BME280 Sensor & I2C
import board
import busio
from adafruit_bme280.basic import Adafruit_BME280_I2C
import sys

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')

        self.get_logger().info("Sensor Node has been started")
        self.sensor1_initialized = False
        self.sensor2_initialized = False

        # Setup I2C bus1 on GPIO 2/3 --> (1, 3, 2)
        try:
            self.i2c_1 = busio.I2C(board.SCL, board.SDA)
            self.get_logger().debug("[DEBUG] I2C_1 initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize I2C_1 using board.SCL/SDA: {e}")
            self.i2c_1 = None

        # Setup I2C on GPIO 0/1 --> (0, 1, 0) 
        try:
            self.i2c_2 = busio.I2C(board.D1, board.D0)
            self.get_logger().debug("[DEBUG] I2C_3 initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize I2C_0 using board.D1/D0: {e}")
            self.i2c_2 = None

        # Initialize sensor 1 (inside bucket)
        if self.i2c_1:
            try:
                self.sensor1 = Adafruit_BME280_I2C(self.i2c_1, address=0x76)
                self.sensor1_initialized = True
                self.get_logger().info("Sensor 1 (Inside Bucket) initialized successfully")
            except Exception as e:
                self.get_logger().error(f"Sensor 1 (Inside Bucket) failed to initialize: {e}")
                self.sensor1 = None

        # Initialize sensor 2 (ambient)
        if self.i2c_2:
            try:
                self.sensor2 = Adafruit_BME280_I2C(self.i2c_2, address=0x76)
                self.sensor2_initialized = True
                self.get_logger().info("Sensor 2 (Ambient) initialized successfully")
            except Exception as e:
                self.get_logger().error(f"Sensor 2 (Ambient) failed to initialize: {e}")
                self.sensor2 = None

        if not self.sensor1_initialized and not self.sensor2_initialized:
            self.get_logger().error("Both sensors failed to initialize.")

        # Publishers for sensor 1 (temperature, humidity, pressure)
        self.temp_pub_1 = self.create_publisher(Temperature, 'sensor1/temperature', 10)
        self.humidity_pub_1 = self.create_publisher(RelativeHumidity, 'sensor1/humidity', 10)
        self.pressure_pub_1 = self.create_publisher(FluidPressure, 'sensor1/pressure', 10)

        # Publishers for sensor 2 (temperature, humidity, pressure)
        self.temp_pub_2 = self.create_publisher(Temperature, 'sensor2/temperature', 10)
        self.humidity_pub_2 = self.create_publisher(RelativeHumidity, 'sensor2/humidity', 10)
        self.pressure_pub_2 = self.create_publisher(FluidPressure, 'sensor2/pressure', 10)

        # Timer to publish data every 5 seconds
        communication_rate = 5.0  # seconds between each publish
        self.timer = self.create_timer(communication_rate, self.publish_data)

  # Publish sensor data for both sensors (temperature, humidity, pressure)
    def publish_data(self):
        # --- Sensor 1 --- (Inside Bucket)
        if self.sensor1.temperature is not None:  # Ensure sensor is connected before publishing
            msg_temp_1 = Temperature()
            msg_temp_1.temperature = self.sensor1.temperature
            self.temp_pub_1.publish(msg_temp_1)

            msg_humidity_1 = RelativeHumidity()
            msg_humidity_1.relative_humidity = self.sensor1.humidity / 100.0
            self.humidity_pub_1.publish(msg_humidity_1)

            msg_pressure_1 = FluidPressure()
            msg_pressure_1.fluid_pressure = self.sensor1.pressure
            self.pressure_pub_1.publish(msg_pressure_1)
        else:
            self.get_logger().warning("Sensor 1 is not publishing data due to no connection")
        
        # --- Sensor 2 --- (Ambient)
        if self.sensor2.temperature is not None:  # Ensure sensor is connected before publishing
            msg_temp_2 = Temperature()
            msg_temp_2.temperature = self.sensor2.temperature
            self.temp_pub_2.publish(msg_temp_2)

            msg_humidity_2 = RelativeHumidity()
            msg_humidity_2.relative_humidity = self.sensor2.humidity / 100.0
            self.humidity_pub_2.publish(msg_humidity_2)

            msg_pressure_2 = FluidPressure()
            msg_pressure_2.fluid_pressure = self.sensor2.pressure
            self.pressure_pub_2.publish(msg_pressure_2)
        else:
            self.get_logger().warning("Sensor 2 is not publishing data due to no connection")




        
