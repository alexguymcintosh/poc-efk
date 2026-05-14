from setuptools import find_packages, setup

package_name = 'pico_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pico_bridge.launch.py']),
        ('share/' + package_name + '/config', ['config/ekf.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mantis',
    maintainer_email='tyfytr@tutamail.com',
    description='Bridge between Raspberry Pi Pico 2 (MPU-6050 + NEO-M9N) and ROS2 Jazzy.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_parser_node = pico_bridge.serial_parser_node:main',
            'imu_node = pico_bridge.imu_node:main',
            'gps_node = pico_bridge.gps_node:main',
            'gps_to_odom_node = pico_bridge.gps_to_odom_node:main',
        ],
    },
)
