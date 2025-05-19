from setuptools import find_packages, setup

package_name = 'logger_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='Chris',
    maintainer_email='s377397@oslomet.no',
    description='Logger node for capturing and logging data from various components such as fans, RC',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'logger_node = logger_node.logger:main',
        ],
    },
)
