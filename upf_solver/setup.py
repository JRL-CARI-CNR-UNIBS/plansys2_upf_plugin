from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'upf_solver'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'example'), glob('example/*.pddl')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools',
                      'unified-planning[engines]'],
    zip_safe=True,
    maintainer='kalman',
    maintainer_email='samuele.sandrini@polito.it',
    description='Upf Solver ROS2 Node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'upf_solver = upf_solver.upf_solver:main'
        ],
    },
)
