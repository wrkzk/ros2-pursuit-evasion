import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pursuit_evasion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='warren',
    maintainer_email='wrkzk@protonmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'manager = pursuit_evasion.manager:main',
            'pursuit_controller = pursuit_evasion.pursuit_controller:main',
            'evasion_controller = pursuit_evasion.evasion_controller:main',
            'sim_supervisor = pursuit_evasion.supervisor:main',
            'pursuit_controller_voronoi = pursuit_evasion.pursuit_controller_voronoi:main'
        ],
    },
)
