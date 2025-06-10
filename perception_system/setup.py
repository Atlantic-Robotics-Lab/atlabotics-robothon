from setuptools import find_packages, setup

package_name = 'perception_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/board_localization_launch.py'] ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='camillo',
    maintainer_email='0802dipshikhadas@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_system_node = perception_system.perception_system_node:main',
            'realsense_node = perception_system.realsense:main',
            'board_localization_node = perception_system.board_localization:main'
            'detection_node = perception_system.detection_task:main'
        ],
    },
)
