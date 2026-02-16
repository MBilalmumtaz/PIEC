from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'piec_analysis'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install config files
        (os.path.join('share', package_name, 'config'),
         glob('piec_analysis/config.yaml')),
        # Install data templates
        (os.path.join('share', package_name, 'data/templates'),
         glob('data/templates/*')),
        # Install sample data
        (os.path.join('share', package_name, 'data/sample_data'),
         glob('data/sample_data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muhammad Amjad',
    maintainer_email='amjad@todo.todo',
    description='Analysis and visualization package for PIEC experimental results',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate_all_results = piec_analysis.scripts.generate_all_results:main',
            'generate_pinn_figures = piec_analysis.scripts.generate_pinn_figures:main',
            'generate_navigation_figures = piec_analysis.scripts.generate_navigation_figures:main',
            'generate_comparison_tables = piec_analysis.scripts.generate_comparison_tables:main',
            'generate_ablation_study = piec_analysis.scripts.generate_ablation_study:main',
        ],
    },
)
