import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'alfan_webots_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),
        # Include all world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.*')),
        # Include all proto files
        (os.path.join('share', package_name, 'protos'), glob('protos/*.proto')),
        # Include all asset files
        (os.path.join('share', package_name, 'protos/alfan_2025/assets'), glob('protos/alfan_2025/assets*.*')),
        (os.path.join('share', package_name, 'protos/alfan_lower_body_only/assets'), glob('protos/alfan_lower_body_only/assets/*.*')),
        (os.path.join('share', package_name, 'protos/alfan_2025_fixed_stl'), glob('protos/alfan_2025_fixed_stl/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aizar',
    maintainer_email='aizarhafizh@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Make sure the path to the executable is correct
            # 'alfan_webots_driver = alfan_webots_sim.alfan_webots_driver:main'
            'sim_launcher = alfan_webots_sim.sim_launcher:main',
            'single_robot = alfan_webots_sim.single_robot:main'
        ],
    },
)