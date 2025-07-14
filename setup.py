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
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
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
            'nav0 = asv_arov_navigation.navigation0:main',
            'nav0_sim = asv_arov_navigation.nav0_sim_supporter:main'
        ],
    },
)
