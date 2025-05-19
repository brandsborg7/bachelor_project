from setuptools import find_packages, setup

package_name = 'main_node'

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
    description='Main node for controlling and managing multiple nodes such as lift, sensors, RC servo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_node = main_node.main:main',  # Entry point for your main node
        ],
    },
)
