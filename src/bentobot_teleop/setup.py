from setuptools import setup

package_name = 'bentobot_teleop'

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
    description='Joystick teleoperation for BentoBot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bentobot_teleop_node = bentobot_teleop.teleop_node:main',
        ],
    },
)
