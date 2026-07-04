import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'robot_behavior_tree'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='murain',
    maintainer_email='16939672+murain37@user.noreply.gitee.com',
    description='High-level behavior-tree task orchestration.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_task_server = robot_behavior_tree.task_server:main',
        ],
    },
)
