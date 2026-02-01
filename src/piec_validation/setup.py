from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'piec_validation'

setup(
    name=package_name,
    version='0.1.0', # Changed from generic 0.0.0
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'], # Added common dependencies
    zip_safe=True,
    maintainer='amjad',
    maintainer_email='amjad@todo.todo',
    description='Metrics collection and validation for the UKF-PIEC framework.',
    license='MIT', # Choose an appropriate license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Format: 'executable_name = package_name.filename:main_function'
            'metrics_collector = piec_validation.metrics_collector:main',
        ],
    },
)
