from setuptools import find_packages, setup

package_name = 'piec_ukf_localization'

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
    maintainer='amjad',
    maintainer_email='amjad@todo.todo',
    description='UKF localization node',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
entry_points={
    'console_scripts': [
        'ukf_node = piec_ukf_localization.ukf_node:main',
    ],
},
)
