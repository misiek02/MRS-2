from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mrs_consensus_improved'

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
    description='This package is an improved version of the base mrs_consensus package, transferrable to hardware',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rendezvous_controller = mrs_consensus_improved.rendezvous_controller:main',
            'formation_controller = mrs_consensus_improved.formation_controller:main',
            'decentralized_sIA = mrs_consensus_improved.decentralized_sIA:main'
        ],
    },
)
