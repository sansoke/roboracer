from setuptools import setup

package_name = 'disparity_extender'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeanho',
    maintainer_email='you@example.com',
    description='Disparity extender node',
    license='',
    entry_points={
    'console_scripts': [
        'disparity_extender = disparity_extender.disparity_extender_node:main',
    ],
}

)
