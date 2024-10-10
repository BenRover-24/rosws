from setuptools import setup
import os
from glob import glob

package_name = 'benrover_assembly'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), ['launch/launch_simulation.py']),  # <--  Chemin explicite vers le fichier launch 
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL'))
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elfriedkinzoun',
    maintainer_email='ekfriedkinzoun@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
