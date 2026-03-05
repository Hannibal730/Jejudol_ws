from setuptools import find_packages, setup

package_name = 'simulator_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='ryuu',
    maintainer_email='ryuu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
entry_points={
    'console_scripts': [
        'pub_auto_steering_angle = simulator_pkg.pub_auto_steering_angle:main',
        'pub_auto_throttle = simulator_pkg.pub_auto_throttle:main',
        'serial_bridge = simulator_pkg.serial_bridge:main',
    ],
},

)
