from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'par_snc'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your.email@student.rmit.edu.au',
    description='PAR Search and Navigation Challenge package',
    license='RMIT IP - Not for public release',
    entry_points={
        'console_scripts': [
            'navigation_node = par_snc.navigation_node:main',
            'hazard_detector = par_snc.hazard_detector:main',
            'hazard_detection_node = par_snc.hazard_detection:main',
            'path_node = par_snc.path_node:main',
        ],
    },
)