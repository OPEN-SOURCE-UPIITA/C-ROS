from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'carro_simulacion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
	(os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
	(os.path.join('share', package_name, 'materials/textures'), glob('materials/textures/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jonathan Jason Medina Martinez',
    maintainer_email='jason240208@gmail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sim_adapter = carro_simulacion.sim_adapter:main',
        ],
    },
)
