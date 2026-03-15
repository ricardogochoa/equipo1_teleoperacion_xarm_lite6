from setuptools import find_packages, setup

package_name = 'ws_master_bridge'

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
    maintainer='ochoagram',
    maintainer_email='ochoagram@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
   entry_points={
    'console_scripts': [
        'master_ws_bridge = ws_master_bridge.master_ws_bridge:main',
        'test_traj_pub = ws_master_bridge.test_traj_pub:main',
    ],
},
)
