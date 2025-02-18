from setuptools import setup

package_name = 'ros2_roboclaw'
submodules = 'src'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ZGES',
    maintainer_email='piotrpassternak@gmail.com',
    description='ROS2 drivers for roboclaw v.3.1.3',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'steer = ros2_roboclaw.roboclaw_node:main',
        ],
    },
)
