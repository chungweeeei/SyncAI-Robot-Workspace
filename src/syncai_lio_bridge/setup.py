import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'syncai_lio_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='chungweeeei@gmail.com',
    description='Bridges FAST-LIO2 3D localization onto the wheel-odom TF '
                'chain (publishes the AMCL-style map->odom correction)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lio_bridge_node = syncai_lio_bridge.lio_bridge_node:main',
        ],
    },
)
