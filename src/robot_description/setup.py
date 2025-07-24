from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_description'

def get_files_in(directory):
    return [f for f in glob(os.path.join(directory, '*')) if os.path.isfile(f)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), get_files_in('urdf')),
        (os.path.join('share', package_name, 'urdf', 'assets'), glob('urdf/assets/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ian',
    maintainer_email='ianshi1026@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
