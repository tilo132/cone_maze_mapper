from setuptools import setup, find_packages

package_name = 'cone_tracking'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Basic Cone Trcking with Yolo and',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracking = '+package_name+'.'+package_name+':main',
            'pnp = '+package_name+'.pnp:main',
            'lidar_bbox = '+package_name+'.lidar_bbox:main',
        ],
    },
)
