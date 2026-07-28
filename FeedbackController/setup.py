from setuptools import find_packages, setup

package_name = 'feedback_controller'

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
    maintainer='phri1',
    maintainer_email='bui@cip.ifi.lmu.de',
    description='Multimodal feedback: Home Assistant lights, audio speaker feedback, Kinova arm gestures',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'feedback_node = FeedbackController.feedback_node:main',
        ],
    },
)
