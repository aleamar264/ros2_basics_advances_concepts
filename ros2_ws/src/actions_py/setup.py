from setuptools import setup
from glob import glob
import os

package_name = 'actions_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arthemis',
    maintainer_email='alejandroamar66@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "count_until_server=actions_py.count_until_server:main",
            "count_until_client=actions_py.count_until_client:main"
        ],
    },
)
