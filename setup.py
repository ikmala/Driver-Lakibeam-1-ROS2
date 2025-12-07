from setuptools import setup

package_name = 'lakibeam1_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lakibeam1_driver.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ikmal',
    maintainer_email='ikmalalais@gmail.com',
    description='Driver Lakibeam 1 ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lakibeam1_node = lakibeam1_driver.lakibeam1_node:main',
        ],
    },
)

