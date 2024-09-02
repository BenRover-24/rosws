from setuptools import find_packages, setup

package_name = 'benrover_actuator'

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
    maintainer='elfriedkinzoun',
    maintainer_email='ekfriedkinzoun@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = benrover_actuator.actuator_node:main',
            'servo_control_node = benrover_actuator.servo_control_node:main'  
        ],
    },
)