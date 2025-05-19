from setuptools import find_packages, setup

package_name = 'fan_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','rclpy'],
    zip_safe=True,
    maintainer='Chris',
    maintainer_email='s377397@oslomet.no',
    description='Fan node for controlling a fan using GPIO on Raspberry Pi',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fan_node = fan_node.fan:main',  # Entry point for your fan node
        ],
    },
)




