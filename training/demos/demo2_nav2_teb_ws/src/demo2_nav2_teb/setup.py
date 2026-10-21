from setuptools import setup
from glob import glob
import os

package_name = 'demo2_nav2_teb'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml') + glob('config/*.csv')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    entry_points={'console_scripts': [
        'path_source = demo2_nav2_teb.path_source:main',
        'follow_path_client = demo2_nav2_teb.follow_path_client:main',
    ]},
)
