from setuptools import find_packages, setup

package_name = 'time_msgs_rclpy_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minsu01',
    maintainer_email='minsu01@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'time_publisher = time_msgs_rclpy_pkg.time_publisher:main',
            'massage_time = time_msgs_rclpy_pkg.massage_time:main',
        ],
    },
)
