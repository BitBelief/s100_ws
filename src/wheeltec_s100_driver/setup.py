from glob import glob

from setuptools import find_packages, setup

package_name = 'wheeltec_s100_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='jam4413256@gmail.com',
    description='WHEELTEC S100 差速底盤的最小 ROS 2 驅動',
    license='MIT',
    entry_points={
        'console_scripts': [
            's100_driver = wheeltec_s100_driver.s100_node:main',
        ],
    },
)
