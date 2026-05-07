import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'leo_pid_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ── Required by ROS 2 to find the package ────────────────
        # This registers the package with the ament index so that
        # ros2 run / ros2 launch can find it.
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # ── package.xml ──────────────────────────────────────────
        # Required by colcon. Contains package metadata and
        # dependency declarations.
        ('share/' + package_name, ['package.xml']),

        # ── Launch files ─────────────────────────────────────────
        # glob('launch/*.py') finds all .py files in launch/ and
        # installs them to share/leo_pid_demo/launch/. This is
        # how ros2 launch finds master_launch.py.
        ('share/' + package_name + '/launch',
            glob('launch/*.py')),

        # ── RViz config ──────────────────────────────────────────
        # Installs the .rviz file so the launch file can find it
        # using get_package_share_directory().
        ('share/' + package_name,
            glob('*.rviz')),

        # ── Scripts ──────────────────────────────────────────────
        # Installs kill_all.sh so it's accessible after building.
        ('share/' + package_name + '/scripts',
            glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ishi',
    maintainer_email='ishitasinghal2000@gmail.com',
    description='PID waypoint follower for Leo Rover in Gazebo',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pid_waypoint_follower = leo_pid_demo.pid_controller:main',
        ],
    },
)