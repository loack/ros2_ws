import os
from glob import glob
from setuptools import setup
from setuptools import find_packages, setup

package_name = 'kuka_urdf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),glob('launch/*')),
        (os.path.join('share', package_name, 'urdf'),glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes','visual'),glob('meshes/visual/*')),
        (os.path.join('share', package_name, 'meshes','collision'),glob('meshes/collision/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lolo',
    maintainer_email='lolo@todo.todo',
    description='urdf viewer of kukabot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = kuka_urdf.state_publisher:main',
            'hello_node = kuka_urdf.hello_node:main',
        ],
    },
)
