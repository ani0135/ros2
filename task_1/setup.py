from setuptools import setup
from glob import glob
import os

package_name = 'task_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # Include world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        # Optionally, include RViz configuration files
        # (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aniket',
    maintainer_email='t23189@students.iitmandi.ac.in',
    description='Package for camera integration',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = task_1.camera_publisher:main',
            'image_subscriber = task_1.image_subscriber:main',
        ],
    },
)
