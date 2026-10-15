from setuptools import setup
from glob import glob

package_name = 'simulation_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('share/' + package_name + '/*.xml')),
        ('share/' + package_name + '/meshes', glob('share/' + package_name + '/meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bai',
    maintainer_email='2774311929@qq.com',
    description='MuJoCo simulation node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'simulation_node = simulation_description.simulation_node:main',
        ],
    },
)
