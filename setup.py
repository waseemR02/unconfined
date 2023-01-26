from setuptools import setup

package_name = 'unconfined'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='waseenR02',
    maintainer_email='waseem.riaz.9999@gmail.com',
    description='ROS2 package for controlling usb_cam and taking panoramic photo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_to_servo = unconfined.joy_to_servo:main',
            'servo_to_spine = unconfined.servo_to_spine:main',
            'view_stream = unconfined.view_stream:main'
        ],
    },
)
