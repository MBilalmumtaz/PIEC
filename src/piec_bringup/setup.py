from setuptools import setup
import os
from glob import glob

package_name = 'piec_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # PROPER WAY to include launch files:
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amjad',
    maintainer_email='amjad@todo.todo',
    description='PIEC Bringup Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'test_goal_publisher = piec_bringup.test_goal_publisher:main',
        'debug_path = piec_bringup.debug_path:main',
        'tf_validator = piec_bringup.tf_validator:main',
    ],
},
)
