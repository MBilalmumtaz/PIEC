from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'piec_pinn_surrogate'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install models directory
        (os.path.join('share', package_name, 'models'), 
         glob('models/*.pt')),
        # Install launch files if any
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Physics-Informed Neural Network Surrogate Model for UKF-PIEC',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pinn_service = piec_pinn_surrogate.pinn_service:main',
            'train_pinn = piec_pinn_surrogate.train_pinn:main',
        ],
    },
)
