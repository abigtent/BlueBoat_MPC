from setuptools import find_packages, setup

package_name = 'vessel_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    package_data={
        package_name: ['waypoints.txt'],
    },
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),

],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='henrik',
    maintainer_email='henrik@todo.todo',
    description='NMPC-based vessel control using acados',
    license='MIT',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nmpc_node = vessel_control.nmpc_node:main',
        ],
    },
)

