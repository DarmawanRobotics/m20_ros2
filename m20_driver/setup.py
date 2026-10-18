from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'm20_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'test'), ['test/fake_robot.py']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Agus Darmawan',
    maintainer_email='darmawandeveloper@gmail.com',
    description='ROS 2 driver for the Deep Robotics Lynx M20 (inspection protocol over UDP)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'driver_node = m20_driver.driver_node:main',
        ],
    },
)
