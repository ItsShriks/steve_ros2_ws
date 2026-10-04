import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'steve_manipulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.srdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'tools'), glob('tools/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ItsShriks',
    maintainer_email='nakhyeshrikar@icloud.com',
    description='Arm + gripper control for Steve (UR5e + Robotiq 2F-85) with MoveIt 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manipulator = steve_manipulation.manipulator:main',
        ],
    },
)
