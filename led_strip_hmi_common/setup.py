from setuptools import find_packages, setup

package_name = 'led_strip_hmi_common'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    install_requires=[
        'setuptools',
        'pyyaml',
        'numpy',
        'rclpy',
        'geometry_msgs',
        'tf2_ros',
        'tf2_geometry_msgs',
    ],
    python_requires='>=3.8,<4',
    zip_safe=True,
    maintainer='Marco Wallner',
    maintainer_email='marco.wallner@ait.ac.at',
    author='Marco Wallner',
    author_email='marco.wallner@ait.ac.at',
    url='https://github.com/AIT-Assistive-Autonomous-Systems/led_strip_hmi',
    description='Shared utility library for loading and validating LED-strip configurations, '
    'performing coordinate transforms, and representing virtual strip geometry.',
    license='Apache-2.0',
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: Apache Software License',
        'Operating System :: OS Independent',
    ],
    data_files=[
        # ament index marker
        (
            'share/ament_index/resource_index/packages',
            ['ament_index/resource_index/packages/led_strip_hmi_common'],
        ),
        # package.xml for ament
        ('share/led_strip_hmi_common', ['package.xml']),
    ],
    tests_require=['pytest', 'pytest-flake8', 'pytest-pep257'],
)
