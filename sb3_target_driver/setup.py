from setuptools import find_packages, setup

package_name = 'sb3_target_driver'

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
    maintainer_email='y.braendle@th-bingen.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver_inference = sb3_target_driver.driver_inference:main',
            'twist_test = sb3_target_driver.twist_test:main',
        ],
    },
)
