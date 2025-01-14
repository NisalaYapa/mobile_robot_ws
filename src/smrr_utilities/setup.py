from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'smrr_utilities'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(package_name, "smrr_utilities/include"),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nisala',
    maintainer_email='nisala.vihanga28@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
