from setuptools import find_packages, setup

package_name = 'smrr_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'PyQt5'],
    zip_safe=True,
    maintainer='nisala',
    maintainer_email='106132194+NisalaYapa@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'test_gui= smrr_gui.test_gui:main'
        ],
    },
)
