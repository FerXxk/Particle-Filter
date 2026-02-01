from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pf_adr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'models'), glob('models/*.sdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roberto',
    maintainer_email='robmorent@alum.us.es',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'particle_filter_node = pf_adr.particle_node:main', # nodo del filtro de particulas 1
            'beacon_node = pf_adr.beacon:main', # nodo de la baliza
            'particle_filter_node2 = pf_adr.particle_node_2:main',  # nodo del filtro de particulas smart
            'ekf_filter = pf_adr.ekf_filter:main',  #nodo del filtro ekf
            'particle_filter_node_ekf = pf_adr.particle_node_ekf:main', # nodo del filtro de particulas con ekf
            'beacon_activity_control = pf_adr.beacon_activity_control:main', # nodo de control de actividad de la baliza para filtro smart
        ],
    },
)
