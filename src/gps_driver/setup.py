from setuptools import setup

package_name = 'gps_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/standalone_driver.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aniketfasate',
    maintainer_email='fasateaniket5@gmail.com',
    description='GNSS driver',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # left side is the executable name; right side is module:function
            'driver = gps_driver.driver:main',
            'gps_driver_node = gps_driver.driver:main',
        ],
    },
)
