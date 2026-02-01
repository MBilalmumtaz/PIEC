from setuptools import find_packages, setup

package_name = 'piec_path_optimizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),  # ← this already includes new files
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amjad',
    maintainer_email='amjad@todo.todo',
    description='Path optimization using PINN surrogate and NSGA-II',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_optimizer = piec_path_optimizer.path_optimizer:main',
            'complete_path_optimizer = piec_path_optimizer.complete_path_optimizer:main',

        ],
    },
)

