from setuptools import setup, find_packages
import glob
import os

package_name = 'led_strip_hmi_projector'
config_files = glob.glob(os.path.join('config', '*.yaml'))
launch_files = glob.glob(os.path.join('launch', '*.launch.py'))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    install_requires=[
        'setuptools',
        'pyyaml',
        'numpy',
        'opencv-python',
    ],
    python_requires='>=3.8,<4',
    zip_safe=True,
    maintainer='Marco Wallner',
    maintainer_email='marco.wallner@ait.ac.at',
    author='Marco Wallner',
    author_email='marco.wallner@ait.ac.at',
    url='https://github.com/AIT-Assistive-Autonomous-Systems/led_strip_hmi',
    description=(
        'Projects 3D detections (vision_msgs/Detection3DArray or sensor_msgs/LaserScan) '
        'onto virtual LED-strip geometry, publishing LED indices and debug images.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'projector_node = led_strip_hmi_projector.projector_node:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['ament_index/resource_index/packages/led_strip_hmi_projector']),
        ('share/led_strip_hmi_projector', ['package.xml']),
        ('share/led_strip_hmi_projector/config', config_files),
        ('share/led_strip_hmi_projector/launch', launch_files),
    ],
)
