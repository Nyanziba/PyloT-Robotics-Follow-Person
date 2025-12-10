from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'PyLoT-Robotics-Follow-Person'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nyanziba',
    maintainer_email='126231924+Nyanziba@users.noreply.github.com',
    description='The package for following a person using Yolov8-pose model and Navigation2',
    license='AGPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'realsense_track = PyLoT-Robotics-Follow-Person.realsense_track:main',
            'person_to_goalupdate = PyLoT-Robotics-Follow-Person.person_to_goalupdate:main',
        ],
    },
)
