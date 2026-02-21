from setuptools import find_packages, setup

package_name = 'car_py_pkg'

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
    maintainer='mahmudch',
    maintainer_email='mahmudch12@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          "movement_node = car_py_pkg.movement_node:main",
          "controller_node = car_py_pkg.controller_node:main",
          "gps_xyz_node = car_py_pkg.gps_xyz_node:main",
          "imu_monitor_node = car_py_pkg.imu_monitor_node:main"
        ],
    },
)
