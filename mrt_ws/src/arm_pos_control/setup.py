from setuptools import find_packages, setup

package_name = 'arm_pos_control'

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
    maintainer='harshit',
    maintainer_email='harshitsomani09@gmail.com',
    description='MRT IITB 2025: Package for Position Control of Robotic Arm',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'handler = arm_pos_control.input_handler:main',
            'queue = arm_pos_control.input_queue:main',
            'pid_controller = arm_pos_control.pid_controller:main',
        ],
    },
)
