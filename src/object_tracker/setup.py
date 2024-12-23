from setuptools import find_packages, setup

package_name = 'object_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['object_tracker', 'object_tracker.include']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sadeep',
    maintainer_email='sadeepm20@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inference_node = object_tracker.inference_node:main',
            'lidar_subscriber = object_tracker.lidar_subscriber:main',
            'visualizer = object_tracker.visualizer:main',
        ],
    },
)
