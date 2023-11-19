from setuptools import setup

package_name = 'challenge_actions'

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
            "move_robot_server= challenge_actions.robot_server:main",
            "move_robot_client= challenge_actions.robot_cliente:main"
        ],
    },
)
