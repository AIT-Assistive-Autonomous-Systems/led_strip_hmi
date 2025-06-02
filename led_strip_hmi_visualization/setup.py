from setuptools import setup, find_packages
import glob
import os

package_name = 'led_strip_hmi_visualization'
config_files = glob.glob(os.path.join('config', '*.yaml'))
launch_files = glob.glob(os.path.join('launch', '*.launch.py'))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    install_requires=[
        'setuptools',
        'pyyaml',
    ],
    python_requires='>=3.8,<4',
    zip_safe=True,
    maintainer='Marco Wallner',
    maintainer_email='marco.wallner@ait.ac.at',
    author='Marco Wallner',
    author_email='marco.wallner@ait.ac.at',
    url='https://github.com/AIT-Assistive-Autonomous-Systems/led_strip_hmi',
    description=(
        'Visualize virtual LED strips in RViz by rendering static outlines '
        'and dynamic highlights driven by projection results.'
    ),
    license='Apache-2.0',
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: Apache Software License',
        'Operating System :: OS Independent',
    ],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'strip_visualizer = led_strip_hmi_visualization.strip_visualizer:main',
        ],
    },
    data_files=[
        # ament index marker
        ('share/ament_index/resource_index/packages',
            ['ament_index/resource_index/packages/led_strip_hmi_visualization']),
        # package.xml
        ('share/led_strip_hmi_visualization', ['package.xml']),
        # config files
        ('share/led_strip_hmi_visualization/config', config_files),
        # launch files
        ('share/led_strip_hmi_visualization/launch', launch_files),
    ],
)
