from setuptools import setup

package_name = 'bentobot_mcu_bridge'

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
    description='Serial bridge between ROS and the BentoBot Teensy',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bentobot_mcu_bridge_node = bentobot_mcu_bridge.mcu_bridge_node:main',
        ],
    },
)
