"""Launch balance robot"""

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

from launch import LaunchDescription  # noqa: E402
from launch import LaunchIntrospector  # noqa: E402
from launch import LaunchService  # noqa: E402

import launch.actions  # noqa: E402
import launch_ros.actions  # noqa: E402

def generate_launch_description():

    lqr_controller = launch_ros.actions.Node(package='balance_robot', executable='lqr_controller', output='screen', on_exit=launch.actions.Shutdown())
    orientation = launch_ros.actions.Node(package='balance_robot', executable='orientation', arguments=['-i', 'lsm'], output='screen', on_exit=launch.actions.Shutdown())
    motor_controller = launch_ros.actions.Node(package='balance_robot_py', executable='motor_controller', output='screen', on_exit=launch.actions.Shutdown())
    serial_publisher = launch_ros.actions.Node(package='balance_robot_rpi', executable='serial_publisher', name='serial_publisher', output='screen', on_exit=launch.actions.Shutdown())

    return LaunchDescription([
        lqr_controller,
        orientation,
        motor_controller,
        serial_publisher,
    ])

def main(argv=sys.argv[1:]):
    ld = generate_launch_description()

    print('Starting introspection of launch description...')
    print('')

    print(LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    # ls = LaunchService(debug=True)
    ls = LaunchService()
    ls.include_launch_description(ld)
    return ls.run()

if __name__ == '__main__':
    main()