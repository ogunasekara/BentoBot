from setuptools import setup

package_name = 'bentobot_laser'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Oshadha Gunasekara',
    maintainer_email='oshadha123@gmail.com',
    description='LaserScan processing for the BentoBot LD06 lidar',
    license='MIT',
    entry_points={
        'console_scripts': [
            'scan_resampler_node = bentobot_laser.scan_resampler_node:main',
        ],
    },
)
