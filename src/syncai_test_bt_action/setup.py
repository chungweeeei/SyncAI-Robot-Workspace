from setuptools import find_packages, setup

package_name = 'syncai_test_bt_action'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='chungweeeei@gmail.com',
    description='Python test node exercising syncai behavior tree actions',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_bt_action = syncai_test_bt_action.test_bt_action:main',
            'cancel_client = syncai_test_bt_action.cancel_client:main',
        ],
    },
)
