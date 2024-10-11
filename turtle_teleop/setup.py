from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'turtle_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  # Automatically find all packages
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ani',
    maintainer_email='t23189@students.iitmandi.ac.in',
    description='Turtle move with keyboard and draw three circles',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tele-op = turtle_teleop.turtle_tele_op:main',
            'draw_circles = turtle_teleop.draw_circles:main',  # Ensure this matches your file name
        ],
    },
)
