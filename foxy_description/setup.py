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
