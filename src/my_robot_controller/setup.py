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
    maintainer='qcc',
    maintainer_email='qcc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={ #this is an arrayy, don't forget comma
        'console_scripts': [
            "second_node = my_robot_controller.my_first_node:main", # /test_node: ros2 exectue table /my_robot_controller: the directory name  /my_first_node:the file name /main: the function we want to run
            "draw_circle = my_robot_controller.draw_circle:main",
            "grid_spiral = my_robot_controller.grid_spiral:main",
            "test_grid_spiral = my_robot_controller.test_grid_spiral:main",
            "testHazard = my_robot_controller.testHazard:main",
            "waypoint_spiral = my_robot_controller.test_waypoint_spiral:main",
            "testWaypoint = my_robot_controller.testWaypoint:main"
        ],
    },
)
 