from setuptools import setup
import os
from glob import glob

package_name = 'disparity_extender'

setup(
    name=package_name,
    version='0.0.1',
    # This finds the sub-folder and all .py files inside it
    packages=[package_name],
    
    # This block fixes the build warnings
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@todo.todo',
    description='Disparity Extender Node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This points to the main function in your script
            # <executable_name> = <package_name>.<file_name>:main
            'disparity_extender = disparity_extender.disparity_extender_node:main',
        ],
    },
)