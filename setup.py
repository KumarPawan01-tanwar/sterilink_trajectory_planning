from setuptools import find_packages, setup

package_name = 'sterilink_trajectory_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'nav_msgs',
        'geometry_msgs',
        'derived_object_msgs',
        'ackermann_msgs'
    ],
    zip_safe=True,
    maintainer='Hassan Abdelhadi',
    maintainer_email='has8385s@hs-coburg.de',
    description='Trajectory Planning Node for STERILINK Autonomous Mobile Robot',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'trajectory_planning_node = sterilink_trajectory_planning.trajectory_planning:main',
        ],
    },
)
