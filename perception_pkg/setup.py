from setuptools import find_packages, setup

package_name = 'perception_pkg'

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
        'vision_Nav = perception_pkg.vision_Nav:main',
        'choose_path = perception_pkg.choose_path:main',
        'vfg_navi = perception_pkg.vfg_navi:main',
        'vfg_navi2 = perception_pkg.vfg_navi2:main',
        ],
    },
)
