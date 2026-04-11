from setuptools import find_packages, setup

package_name = 'ugv_control_pkg'

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
    maintainer='asak',
    maintainer_email='abdoorahmansk2004@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apply_torque_from_voltage = ugv_control_pkg.apply_torque_from_voltage:main',
            'publish_states_to_matlab = ugv_control_pkg.publish_states_to_matlab:main',
            'path_generator = ugv_control_pkg.path_generator:main',
        ],
    },
)
