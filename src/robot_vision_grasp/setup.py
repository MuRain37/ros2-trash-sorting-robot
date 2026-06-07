import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'robot_vision_grasp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['models/*']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='murain',
    maintainer_email='16939672+murain37@user.noreply.gitee.com',
    description='Vision based object grasp pipeline for the robot arm.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_grasp_node = robot_vision_grasp.vision_grasp_node:main',
        ],
    },
)
