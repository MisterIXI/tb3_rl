from setuptools import find_packages, setup

package_name = 'sensor_pub'

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
    maintainer='turtle1',
    maintainer_email='turtle1@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'receive_ball = sensor_pub.receive_ball_data:main',
            'sonic_bumper_pub = sensor_pub.sonic_sensor:main',
        ],
    },
)
