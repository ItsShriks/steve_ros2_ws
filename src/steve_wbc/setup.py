import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'steve_wbc'


def tree(src):
    """data_files entries for every file under src, keeping the relative layout."""
    out = {}
    for root, _, files in os.walk(src):
        if files:
            out.setdefault(os.path.join('share', package_name, root), []).extend(
                os.path.join(root, f) for f in files)
    return list(out.items())


setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        *tree('models'),  # MuJoCo model + meshes used as the kinematic model
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ItsShriks',
    maintainer_email='nakhyeshrikar@icloud.com',
    description='Unified velocity-level whole-body controller for Steve (MMO-700 + UR5e)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # MuJoCo / Gazebo / real robot figure-8 WBC
            'steve_figure8_wbc = steve_wbc.steve_figure8_wbc:main',
            # MuJoCo-only demo: table figure-8 with orbiting base + vertical figure-8
            'figure8_wbc_demo = steve_wbc.figure8_wbc:main',
            # active perception (from t=0) + MPC interception + grasp, joint-velocity WBC
            'active_perception_wbc = steve_wbc.active_perception_wbc:main',
        ],
    },
)
