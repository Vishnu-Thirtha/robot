from setuptools import setup
import os

package_name = 'turtlebot_custom'

# automatically collect all files in urdf and launch
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# include launch files
for dirpath, dirnames, filenames in os.walk('launch'):
    for f in filenames:
        data_files.append(('share/' + package_name + '/launch', [os.path.join(dirpath, f)]))


# include urdf files
for dirpath, dirnames, filenames in os.walk('urdf'):
    for f in filenames:
        data_files.append(('share/' + package_name + '/urdf', [os.path.join(dirpath, f)]))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vishnu',
    maintainer_email='you@example.com',
    description='Custom TurtleBot3 with RGB-D sensor',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
