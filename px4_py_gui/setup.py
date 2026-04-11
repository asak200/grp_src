from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'px4_py_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asak',
    maintainer_email='abdoorahmansk2004@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px4_controller_gui = px4_py_gui.px4_controller_gui:main',
            'plotter = px4_py_gui.plotter:main',
        ],
    },
)
