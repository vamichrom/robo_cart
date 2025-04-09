from setuptools import setup
from glob import glob

package_name = 'obj_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vamichrom',
    maintainer_email='vamichrom@gmail.com',
    description='Detects and follows an object on the video stream',
    license='MIT-0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_obj = obj_tracker.detect_obj:main',
            'detect_obj_3d = obj_tracker.detect_obj_3d:main',
            'follow_obj = obj_tracker.follow_obj:main',
        ],
    },
)
