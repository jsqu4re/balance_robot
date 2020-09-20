from setuptools import setup

package_name = 'balance_robot_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'odrive'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='Johannes.Jeising@gmail.com',
    description='Odrive ros wrapper',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = balance_robot_py.motor_controller:main',
            'gain_changer = balance_robot_py.gain_changer:main',
            'xbox_controller = balance_robot_py.xbox_controller:main'
        ],
    },
)
