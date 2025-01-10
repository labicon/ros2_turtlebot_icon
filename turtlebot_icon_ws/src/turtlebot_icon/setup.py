from setuptools import find_packages, setup

package_name = 'turtlebot_icon'

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
    maintainer='penrose512',
    maintainer_email='penrose512@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sub_redis_node = turtlebot_icon.sub_redis_node:main',
            'consensus_node = turtlebot_icon.consensus_node:main',
            'consensus_sub = turtlebot_icon.consensus_sub:main',
            'consensus_pub = turtlebot_icon.consensus_pub:main',
        ],
    },
)
