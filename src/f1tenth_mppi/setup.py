from setuptools import find_packages, setup
from glob import glob 
import os 

package_name = 'f1tenth_mppi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('trajectories/*')),
        ('share/' + package_name, glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cho22',
    maintainer_email='cho22@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mppi = f1tenth_mppi.mppi_node:main',
            'obstacle_detector = f1tenth_mppi.obstacle_detector_node:main',
            'navigation = f1tenth_mppi.navigation_node:main',
            'jax_mppi = f1tenth_mppi.jax_mppi_node:main',
        ],
    },
)
