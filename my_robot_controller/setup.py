from setuptools import find_packages, setup

package_name = 'my_robot_controller'

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
    maintainer='amar81',
    maintainer_email='amar81@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = my_robot_controller.my_first_node:main",
            "draw_circle = my_robot_controller.draw_circle:main",
            "pose_sub = my_robot_controller.pose_sub:main",
            "wrench = my_robot_controller.wrench:main",
            "drone = my_robot_controller.drone:main",
            "drone_stabilizer = my_robot_controller.drone_stabilizer:main",
            "track_bot_controller = my_robot_controller.track_bot_controller:main"
        ],
    },
)
