from setuptools import find_packages, setup

package_name = 'ups'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'smbus2'],
    zip_safe=True,
    maintainer='ronaldo',
    maintainer_email='ronaldo.rjr@gmail.com',
    description='UPS battery state publisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_state_node=ups.battery_state_node:main',
            'ups_status_node=ups.ups_status_node:main',
        ],
    },
)
