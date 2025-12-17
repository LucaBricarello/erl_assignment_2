from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'erl_assignment_2'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Path ai file PDDL
    domain_path = os.path.join(pkg_share, 'pddl', 'assignment2_domain.pddl')
    problem_path = os.path.join(pkg_share, 'pddl', 'assignment2_problem.pddl')

    # 1. Lancia PlanSys2
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_distributed.py')),
        launch_arguments={
            'model_file': domain_path,
            'problem_file': problem_path,
            'namespace': ''
            }.items())

    # 2. Nodi Azione
    move_cmd = Node(
        package=pkg_name,
        executable='action_move',
        name='move_action_node',
        output='screen',
        parameters=[{'action_name': 'move'}] # Match con PDDL
    )

    rotate_cmd = Node(
        package=pkg_name,
        executable='action_rotate',
        name='rotate_action_node',
        output='screen',
        parameters=[{'action_name': 'rotate_and_detect'}] # Match con PDDL
    )
    
    analyze_cmd = Node(
        package=pkg_name,
        executable='action_analyze',
        name='analyze_action_node',
        output='screen',
        parameters=[{'action_name': 'analyze_marker'}] # Match con PDDL
    )

    ld = LaunchDescription()
    ld.add_action(plansys2_cmd)
    ld.add_action(move_cmd)
    ld.add_action(rotate_cmd)
    ld.add_action(analyze_cmd)

    return ld