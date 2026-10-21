from glob import glob
from setuptools import setup


package_name = 'qr_screen_display'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='External screen display for QR and Chinese image-to-text results',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'qr_screen_display = qr_screen_display.qr_screen_display_node:main',
            'ring_photo_client = qr_screen_display.ring_photo_client_node:main',
        ],
    },
)
