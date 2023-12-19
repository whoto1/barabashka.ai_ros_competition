import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'autorace_core_vvv'



setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
     (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
     (os.path.join('share', package_name, 'calibration'), glob('calibration/*'))
   ],
    install_requires=['setuptools',
                      'ultralytics',],
    zip_safe=True,
    maintainer='alina',
    maintainer_email='a.ten3@g.nsu.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_tracking=autorace_core_vvv.path_tracking:main',
            'take_picture=autorace_core_vvv.take_picture:main',
            'pid_controller=autorace_core_vvv.pid_controller:main'
        ],
    },
)
