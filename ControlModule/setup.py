from setuptools import setup, find_packages

package_name = 'control_module'   # must match <name> in package.xml exactly

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
    entry_points={'console_scripts': [
        'motion_test = control_module.MotionTestNode:main',
    ]},
)