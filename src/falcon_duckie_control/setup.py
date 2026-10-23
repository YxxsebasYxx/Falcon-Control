from setuptools import find_packages, setup

package_name = 'falcon_duckie_control'

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
    maintainer='sebastian',
    maintainer_email='sebastian@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
    'arduino_bridge = falcon_duckie_control.arduino_bridge:main',
    'keyboard_teleop = falcon_duckie_control.keyboard_teleop:main',
    'camera_target_detector = falcon_duckie_control.camera_target_detector:main',
    'target_controller = falcon_duckie_control.target_controller:main',
    'drone_detection_control = falcon_duckie_control.drone_detection_control:main',
    'detection_inf = falcon_duckie_control.target_control:main',
        
        

],

    },
)
