import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'g1_task'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='adamlwatts@msn.com',
    description='G1 robot IK task system',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ik_service = g1_task.ik_service:main',
            'joint_trajectory_test = g1_task.joint_trajectory_test:main',
            'task_sequencer = g1_task.task_sequencer:main',
        ],
    },
)
