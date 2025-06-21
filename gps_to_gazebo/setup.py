from setuptools import setup

package_name = 'gps_to_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/gps_to_gazebo']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyproj'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Convert GPS fix to Gazebo XYZ coordinates',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'gps_to_gazebo_node = gps_to_gazebo.gps_to_gazebo_node:main',
        ],
    },
)
