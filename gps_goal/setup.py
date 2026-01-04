from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gps_goal'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Include csv files
        (os.path.join('share', package_name, 'waypoints'), glob('waypoints/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shubh',
    maintainer_email='shubh06kesar@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gps_goal_manager = gps_goal.gps_goal_manager:main',
        ],
    },
)
