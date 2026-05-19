import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_nav2_behaviors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='murain',
    maintainer_email='16939672+murain37@user.noreply.gitee.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'coverage_path_generator = robot_nav2_behaviors.coverage_path_generator:main',
            'coverage_marker_publisher = robot_nav2_behaviors.coverage_marker_publisher:main',
            'map_rectangle_selector = robot_nav2_behaviors.map_rectangle_selector:main',
            'coverage_task_manager = robot_nav2_behaviors.coverage_task_manager:main',
            'coverage_navigator = robot_nav2_behaviors.coverage_navigator:main',
        ],
    },
)
