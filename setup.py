from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'erl_assignment_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Installa i file PDDL
        (os.path.join('share', package_name, 'pddl'), glob('pddl/*.pddl')),

        # Installa i file di launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        # Installa i file URDF e XACRO
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        
        # Installa i mondi Gazebo
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        
        # Installa le configurazioni Rviz
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        
        # Installa config
        (os.path.join('share', package_name, 'config'), glob('config/*')),

        # meshes:
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='luca.bricarello@outlook.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'action_move = erl_assignment_2.action_move:main',
        'action_rotate = erl_assignment_2.action_rotate:main',
        'action_analyze = erl_assignment_2.action_analyze:main',
        'get_plan_and_execute = erl_assignment_2.get_plan_and_execute:main',
    ],
},
)
