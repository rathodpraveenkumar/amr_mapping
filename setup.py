from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'amr_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/resource', glob('resource/*')),       
        ('share/' + package_name + '/worlds', glob('worlds/*.wbt')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rathod',
    maintainer_email='rathodpraveenkumar789@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ['navigator = amr_mapping.navigator:main',
        ],
    },
)
