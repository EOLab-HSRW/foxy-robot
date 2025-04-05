import glob
from setuptools import find_packages, setup

package_name = 'foxy_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob.glob('urdf/**/*', recursive=True)),
        ('share/' + package_name + '/launch', glob.glob('launch/*')),
        ('share/' + package_name + '/sensors', glob.glob('sensors/*')),
        ('share/' + package_name + '/worlds', glob.glob('worlds/*')),
        ('share/' + package_name + '/assets/duck', glob.glob('assets/duck/*')),
        ('share/' + package_name + '/assets/small_loop', glob.glob('assets/small_loop/*')),
        ('share/' + package_name + '/assets/large_map', glob.glob('assets/large_map/*')),
        ('share/' + package_name + '/assets/straight_lane', glob.glob('assets/straight_lane/*')),
        ('share/' + package_name + '/assets/small_map', glob.glob('assets/small_map/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harley',
    maintainer_email='harley.lara@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
