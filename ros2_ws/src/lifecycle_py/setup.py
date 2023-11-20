from setuptools import setup

package_name = 'lifecycle_py'

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
            "number_publisher=lifecycle_py.number_publisher:main",
            "number_publisher_lifecylce=lifecycle_py.number_publisher_lifecycle:main",
            "lifecycle_manager_node = lifecycle_py.lifecycle_manager_node:main"
        ],
    },
)
