from setuptools import find_packages, setup

package_name = 'apriltag_ik_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phri9',
    maintainer_email='bui@cip.ifi.lmu.de',
    description='Standalone ROS 2 package for AprilTag tracking & IK control using Pinocchio',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ik_mover = apriltag_ik_controller.main:main',
        ],
    },
)
