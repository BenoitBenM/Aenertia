from setuptools import setup
import os
from glob import glob

package_name = 'mybot_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Install launch files
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        # Install config files
        (f'share/{package_name}/config', glob(f'{package_name}/config/*.yaml')),
        # Required for ament index
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nihil',
    maintainer_email='nihil@todo.todo',
    description='Pose saver and MQTT bridge for navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_saver = mybot_nav.pose_saver:main',
            'nav_to_pose = mybot_nav.nav_to_pose:main',
            'mqtt_serial_bridge = mybot_nav.mqtt_serial_bridge:main',  # si tu veux le lancer comme exec
        ],
    },
)
