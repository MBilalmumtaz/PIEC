from setuptools import setup

package_name = 'piec_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amjad',
    maintainer_email='amjad@todo.todo',
    description='PIEC Controller with Obstacle Avoidance',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = piec_controller.controller_node:main',
            'laser_bridge = piec_controller.laser_bridge:main',
            'emergency_stop = piec_controller.emergency_stop:main',
            'obstacle_test = piec_controller.obstacle_test:main',
            'dwa_visualizer = piec_controller.dwa_visualizer:main',
            'magnetometer_converter=piec_controller.magnetometer_converter:main',
        ],
    },
)
