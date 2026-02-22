from setuptools import find_packages, setup
from glob import glob

package_name = 'workcell_webots'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', glob('launch/*launch.py')))
data_files.append(('share/' + package_name + '/worlds', ['worlds/workcell.wbt']))
data_files.append(('share/' + package_name + '/protos', glob('protos/*')))
data_files.append(('share/' + package_name + '/meshes', glob('meshes/*')))
data_files.append(('share/' + package_name + '/config', glob('config/*')))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='daneceni@gmail.com',
    description='Webots simulation package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
