import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'syncai_backend'

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
    maintainer='ros',
    maintainer_email='chungweeeei@gmail.com',
    description='Robot backend node exposing a RESTful API (FastAPI) bridged to ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run syncai_backend backend
            'backend = syncai_backend.backend_node:main',
        ],
    },
)
