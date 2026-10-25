from glob import glob
from setuptools import find_packages, setup

setup(
    name='zsh_navigation',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/zsh_navigation']),
        ('share/zsh_navigation', ['package.xml', 'README.md']),
        ('share/zsh_navigation/launch', glob('launch/*.launch.py')),
        ('share/zsh_navigation/config', glob('config/*.yaml')),
        ('share/zsh_navigation/config/full_race', glob('config/full_race/*.json')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='2097244156',
    maintainer_email='238140893+2097244156@users.noreply.github.com',
    description='ROS 2 competition navigation without a simulator dependency',
    license='Proprietary',
    entry_points={'console_scripts': [
        'full_race_navigation = zsh_navigation.full_race_navigation:main',
        'audit_course = zsh_navigation.audit:main',
    ]},
)
