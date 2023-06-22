from setuptools import setup
import os
from glob import glob


package_name = 'riptide_teleop2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hayden',
    maintainer_email='hgray576@gmail.com',
    description='Package used for the teleoperation of the team\'s robot',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ps3_teleop = riptide_teleop2.ps3_teleop:main',
            "keyboard_teleop = riptide_teleop2.keyboard_teleop:main",
            "xbox_teleop = riptide_teleop2.xbox_teleop:main"
        ],
    },
)
