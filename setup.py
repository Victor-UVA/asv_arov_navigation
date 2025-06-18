from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'asv_arov_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'models'), glob('models/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maddy',
    maintainer_email='abo7fg@virginia.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arov_nav_test = asv_arov_navigation.arov_nav_test:main',
            'arov_nav_test2 = asv_arov_navigation.arov_nav_test2:main',
            'asv_nav_test = asv_arov_navigation.asv_nav_test:main',
            'arov_nav = asv_arov_navigation.arov_nav:main',
            'asv_arov_control_server = asv_arov_navigation.asv_arov_control_server:main',
            'asv_nav = asv_arov_navigation.asv_nav:main',
            'movement_server = asv_arov_navigation.movement_server:main',
            'control_server_test = asv_arov_navigation.control_server_test:main',
            'lifecycle_pose_publisher = asv_arov_navigation.lifecycle_pose_publisher:main'
        ],
    },
)
