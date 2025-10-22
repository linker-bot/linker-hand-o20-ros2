import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'linker_hand_o20'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='linkerhand',
    maintainer_email='linkerhand@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'linker_hand_o20 = linker_hand_o20.linker_hand_o20:main',
            'linker_hand_o20_teleoperated_master = linker_hand_o20.linker_hand_o20_teleoperated_master:main'
        ],
    },
)
