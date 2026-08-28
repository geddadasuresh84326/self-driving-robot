from setuptools import find_packages, setup
import os

package_name = 'robot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), ['camera_calibration.npz'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dileep',
    maintainer_email='geddadasuresh6@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_publisher = robot_vision.camera_publisher:main',
            'camera_capture = robot_vision.camera_capture:main',
            'aruco_marker_detection = robot_vision.aruco_marker_detection:main'
        ],
    },
)
