from glob import glob
from setuptools import setup

package_name = 'talos_mc_executor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='TALOS',
    maintainer_email='daisy.schall@gmail.com',
    description='Workshop MC mode executor ported to the skybeta flight interface.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mc_executor = talos_mc_executor.mc_executor:main',
        ],
    },
)
