from setuptools import setup
from os.path import join
from glob import glob

package_name = 'luos_interface'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial', 'pyluos'],
    zip_safe=True,
    maintainer='Yoan Mollard',
    maintainer_email='yoan@aubrune.eu',
    description='Luos lets you easily connect, interact and program your robots in ROS 1 and ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'broker = luos_interface.broker:main'
        ],
    }
)
