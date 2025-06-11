from setuptools import find_packages, setup

package_name = 'simple_pipeline_service'

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
    maintainer='camillo',
    maintainer_email='0802dipshikhadas@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pipeline_node = simple_pipeline_service.pipeline_node:main',
            'pipeline_node_byod = simple_pipeline_service.pipeline_node_byod:main',
        ],
    },
)
