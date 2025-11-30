from setuptools import find_packages, setup

package_name = 'global_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_file.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ali-elkashef',
    maintainer_email='ali-elkashef@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'random_map = global_planner.random_map:main',
            'starting_point = global_planner.starting_point:main',
            'A_algorithm = global_planner.A_algorithm:main',
        ],
    },
)
