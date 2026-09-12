import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ai_car_web'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'static'), glob('static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='super',
    maintainer_email='hirakawa3.tennis.gogo@gmail.com',
    description='AT-CAR 用 FastAPI Webダッシュボード',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_node = ai_car_web.dashboard_node:main',
            'system_monitor_node = ai_car_web.system_monitor_node:main',
        ],
    },
)
