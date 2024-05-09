import launch

import launch_ros.actions


def generate_launch_description():
    # Create a launch description object
    ld = launch.LaunchDescription()

    # Add nodes to the launch description
    mapping = launch_ros.actions.Node(
        package='mapping',
        executable='mapping',
        name='mapping'
    )
    ld.add_action(mapping)

    nav_move = launch_ros.actions.Node(
        package='chassis_controller',
        executable='nav_goal_move',
        name='nav_goal_move'
    )
    ld.add_action(nav_move)

    pursuit = launch_ros.actions.Node(
        package='controller',
        executable='pursuit_action_server',
        name='pursuit_action_server'
    )
    ld.add_action(pursuit)

    arm_configuration = launch_ros.actions.Node(
        package='arm',
        executable='arm_conf',
        name='arm_conf'
    )
    ld.add_action(arm_configuration)

    arm_detect = launch_ros.actions.Node(
        package='arm',
        executable='arm_detect',
        name='arm_detect'
    )
    ld.add_action(arm_detect)

    b_tree = launch_ros.actions.Node(
        package='brain',
        executable='tree',
        name='tree'
    )
    ld.add_action(b_tree)

    # Add more nodes or actions as needed

    return ld
