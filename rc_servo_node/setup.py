from setuptools import find_packages, setup

package_name = 'rc_servo_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'gpiozero','rclpy'],  # Added gpiozero as a dependency
    zip_safe=True,
    maintainer='Chris',
    maintainer_email='s377397@oslomet.no',
    description='RC Servo node for controlling servos using GPIO on Raspberry Pi',  # Updated descript>    license='MIT',  # Updated license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rc_servo_node = rc_servo_node.rc_servo:main',  # Assuming the entry point is named 'main'        
            ],
    },
)


