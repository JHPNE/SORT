from setuptools import setup, find_packages

package_name = 'vision_module'   # must match <name> in package.xml exactly

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
    entry_points={
        'console_scripts': [
            'tag_detector = vision_module.TagDetectorNode:main',
            'world_space = vision_module.WorldSpaceNode:main',
            'tag_reader = vision_module.TagReaderNode:main',
        ],
    },
)