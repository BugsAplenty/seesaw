from setuptools import setup

package_name = 'seesaw_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@email',
    description='UDP Lidar Receiver',
    license='MIT',
    entry_points={
        'console_scripts': [
            'udp_reader = seesaw_ros2.udp_reader:main',
        ],
    },
)
