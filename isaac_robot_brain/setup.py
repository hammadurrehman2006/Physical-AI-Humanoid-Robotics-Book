from setuptools import setup
import os
from glob import glob

package_name = 'isaac_robot_brain'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include all config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Include all scripts
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hammad Ur Rehman',
    maintainer_email='hammadurrehman@example.com',
    description='NVIDIA Isaac AI Robot Brain with Vision-Language-Action capabilities',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'isaac_robot_brain_node = isaac_robot_brain.main:main',
        ],
    },
)