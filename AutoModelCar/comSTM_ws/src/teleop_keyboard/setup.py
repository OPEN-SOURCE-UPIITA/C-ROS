from setuptools import find_packages, setup

package_name = 'teleop_keyboard'

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
    maintainer='jon',
    maintainer_email='jon@todo.todo',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'key_publisher = teleop_keyboard.key_publisher:main',
            'eight_routine = teleop_keyboard.eight_routine:main',
            'full_test_routine = teleop_keyboard.full_test_routine:main'  # <--- AGREGA ESTA LÍNEA
        ],
    },
)
