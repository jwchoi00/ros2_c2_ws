from setuptools import setup

package_name = 'flask_ros_app'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'flask', 'pytest'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Flask with ROS 2 integration for video feed and service calls',
    license='Apache License 2.0',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_flask_sub_image_with_db = flask_ros_app.ros2_flask_sub_image_with_db:main',
            'ros2_flask_sub_image_with_db_copy = flask_ros_app.ros2_flask_sub_image_with_db_copy:main',
            'ros2_flask_sub_image_with_db_final = flask_ros_app.ros2_flask_sub_image_with_db_final:main',
            'ros2_flask_sub_image_with_db_final_copy = flask_ros_app.ros2_flask_sub_image_with_db_final_copy:main',
            'sub_msgs = flask_ros_app.sub_msgs:main',
            'sqlite3_save = flask_ros_app.sqlite3_save:main',
            'sqlite3_save_copy = flask_ros_app.sqlite3_save_copy:main',
            'sqlite3_save_ver2 = flask_ros_app.sqlite3_save_ver2:main',
            'sqlite3_save_final = flask_ros_app.sqlite3_save_final:main',
            'sqlite3_save_ver3 = flask_ros_app.sqlite3_save_ver3:main',
        ],
    },
)
