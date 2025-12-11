from setuptools import setup
from glob import glob
import os

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.*')) + glob(os.path.join('launch', '*.ini'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.publisher_member_function:main',
            'listener = py_pubsub.subscriber_member_function:main',
            'move2aruco = py_pubsub.move2aruco:main',
            'aruco_localizer = py_pubsub.aruco_localization:main',
            'marker_tf_publisher = py_pubsub.marker_tf_publisher:main',
            'aruco_simulator = py_pubsub.simulator:main',
            'test_localization = py_pubsub.test_localization:main',
        ],
    },
)

