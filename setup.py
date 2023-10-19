from setuptools import setup
from glob import glob
import os

package_name = 'performance_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roar',
    maintainer_email='kaushik.kunal@berkeley.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perf_monitor = performance_monitor.test_performance:main',
            'pref_plotter = performance_monitor.plot_performance:main'
        ],
    },
)
