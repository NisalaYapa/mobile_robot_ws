from setuptools import find_packages, setup

package_name = 'human_data_buffer'

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
    maintainer='ishara',
    maintainer_email='isharadilshanra11@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'human_data_buffer = human_data_buffer.data_buffer:main',
            'human_data_kf = human_data_buffer.human_KF_vel:main',
            'human_data_extracter = human_data_buffer.human_data_extracter:main',
            'human_data_pub = human_data_buffer.test_data_pub:main',
        ],
    },
)
