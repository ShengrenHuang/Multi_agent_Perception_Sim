from setuptools import find_packages, setup

package_name = 'ros_rl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'gymnasium',
        'numpy'
    ],
    zip_safe=True,
    maintainer='cirl',
    maintainer_email='bulin11636@gmail.com',
    description='UAV RL environment using Gymnasium and ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uav_rl_env = ros_rl.UAV_RL_pipeline:main',
        ],
    },
)
