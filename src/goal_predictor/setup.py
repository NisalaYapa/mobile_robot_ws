from setuptools import find_packages, setup

package_name = 'goal_predictor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sithija',
    maintainer_email='sithija.vihanga28@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'crowd_flow_publisher = goal_predictor.crowd_flow_publisher:main',
                'goal_predictor = goal_predictor.goal_predictor:main',
        ],
        },
)
