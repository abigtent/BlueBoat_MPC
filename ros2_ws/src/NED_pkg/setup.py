from setuptools import setup


package_name = 'NED_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Waypoint service package in Python for ROS2.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'waypoint_service_node = NED_pkg.waypoint_service_node:main'
        ],
    },
)