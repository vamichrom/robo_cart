from setuptools import find_packages, setup

package_name = 'usb_gamepad'

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
    maintainer='vamichrom',
    maintainer_email='vamichrom@gmail.com',
    description='Publishes TwistStamped teleop msgs with USB Gamepad',
    license='MIT-0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "usb_gamepad = usb_gamepad.usb_gamepad:main",
        ],
    },
)
