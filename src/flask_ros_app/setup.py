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
            'flask_ros_app = flask_ros_app.app:main',
            'sub_msgs = flask_ros_app.sub_msgs:main',
        ],
    },
)
