from setuptools import setup, find_packages

package_name = 'cypher_eyes'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/cypher_eyes', ['cypher_eyes/best.pt']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'pyserial',
        'numpy',
        'opencv-python',
        'ultralytics',
    ],
    zip_safe=True,
    maintainer='imran',
    maintainer_email='imran@todo.todo',
    description='ROS2 package for object detection and servo tracking',
    license='MIT',
    entry_points={
        'console_scripts': [
            'object_detector = cypher_eyes.detection_node:main',
            'servo_serial = cypher_eyes.tracking_node:main',
        ],
    },
)
