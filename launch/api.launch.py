from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch.actions import RegisterEventHandler, EmitEvent

from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare

from launch.events import matches_action
from launch.event_handlers.on_process_start import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

import lifecycle_msgs.msg
import os

def generate_launch_description():
#Initialize arguments
    declared_arguments = []

    namespace_env = os.environ.get('NAMESPACE', '')

    if namespace_env == "":
        print("A variavel de ambiente NAMESPACE não está configurada!")
        return LaunchDescription()

    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         'namespace',
    #         default_value='',
    #         description='Namespace for the API node.'
    #     )
    # ) 

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Whether use the simulation time.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'api_file',
            default_value=PathJoinSubstitution([FindPackageShare('laser_usv_px4_api'), 'params', 'api.yaml']),
            description='Full path to the file with the all parameters.'
        )
    )

    api_file = LaunchConfiguration('api_file')
    # namespace_arg = LaunchConfiguration('namespace')

    api_lifecycle_node = LifecycleNode(
        package='laser_usv_px4_api',
        executable='api',
        name='px4_api',
        namespace=namespace_env,
        output='screen',
        parameters=[api_file,{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('fmu/in/trajectory_setpoint', ['/', namespace_env, '/fmu/in/trajectory_setpoint']),
            ('fmu/in/vehicle_command',      ['/', namespace_env, '/fmu/in/vehicle_command']),
            ('fmu/in/offboard_control_mode', ['/', namespace_env, '/fmu/in/offboard_control_mode']),
            ('fmu/in/actuator_motors',      ['/', namespace_env, '/fmu/in/actuator_motors']),
            ('fmu/out/actuator_motors',     ['/', namespace_env, '/fmu/out/actuator_motors']),
            ('fmu/out/sensor_combined',     ['/', namespace_env, '/fmu/out/sensor_combined']),
            ('fmu/out/vehicle_odometry',    ['/', namespace_env, '/fmu/out/vehicle_odometry']),
            ('fmu/out/vehicle_control_mode', ['/', namespace_env, '/fmu/out/vehicle_control_mode']),
            ('odometry',                    ['/', namespace_env, '/odometry']),
            ('imu',                         ['/', namespace_env, '/imu']),
            ('actuators',                   ['/', namespace_env, '/actuators']),
            ('arm',                         ['/', namespace_env, '/px4_api/arm']),
            ('disarm',                      ['/', namespace_env, '/px4_api/disarm']),
        ]
    )

    event_handlers = []

    event_handlers.append(
#Right after the node starts, make it take the 'configure' transition.
        RegisterEventHandler(
            OnProcessStart(
                target_action=api_lifecycle_node,
                on_start=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(api_lifecycle_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )),
                ],
            )
        ),
    )

    event_handlers.append(
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=api_lifecycle_node,
                start_state='configuring',
                goal_state='inactive',
                entities=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(api_lifecycle_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            )
        ),
    )

    ld = LaunchDescription()

#Declare the arguments
    for argument in declared_arguments:
        ld.add_action(argument)

#Add client node
    ld.add_action(api_lifecycle_node)

#Add event handlers
    for event_handler in event_handlers:
        ld.add_action(event_handler)

    return ld
