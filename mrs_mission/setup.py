from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mrs_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='lifteddebbymartha@gmail.com',
    description='This package handles mission allocation and control for a group of crazyflies',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'auctioneer_node = mrs_mission.auctioneer_node:main'
            'formation_action = mrs_mission.formation_action:main'
            'bidder_2 = mrs_mission.bidder_2:main'
            'bidder_3 = mrs_mission.bidder_3:main'
            'bidder_4 = mrs_mission.bidder_4:main'
        ],
    },
)
