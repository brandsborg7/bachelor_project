from setuptools import find_packages, setup

package_name = 'sensor_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy'],  # Add any other dependencies
    zip_safe=True,
    maintainer='Chris',
    maintainer_email='s377397@oslomet.no',
    description='A ROS 2 package for reading data from sensors measuring temperature, humidity, and pressure.',
    license='MIT',  # Specify the correct license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = sensor_node.sensor:main',  # Entry point for your sensor node
        ],
    },
)



