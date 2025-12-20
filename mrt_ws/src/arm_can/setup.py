from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arm_can'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'scripts'), 
            glob('scripts/*.sh')),
        (os.path.join('share',package_name,'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='omega',
    maintainer_email='harshitsomani09@gmail.com',
    description='CAN Initialization for Arm Nodes',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arm_can = arm_can.ArmCAN:main'
        ],
    },
)
