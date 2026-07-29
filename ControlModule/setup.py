from setuptools import find_packages, setup

package_name = 'control_module'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phri1',
    maintainer_email='phri1@todo.todo',
    description='MoveIt motion client',
    license='TODO',
    entry_points={
        'console_scripts': [
            'motion_test = control_module.MotionTestNode:main',
            'tag_approach = control_module.TagApproachNode:main',
        ],
    },
)