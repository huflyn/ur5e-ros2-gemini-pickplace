from setuptools import find_packages, setup
from glob import glob

package_name = 'color_detection'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/launch', glob('launch/*launch.[pxy][yma]*')))
data_files.append(('share/' + package_name + '/config', glob('config/*.yaml')))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='daneceni@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'color_detector = color_detection.color_detector:main',
            'hsv_tuner = color_detection.hsv_tuner:main',
        ],
    },
)
