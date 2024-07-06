from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition

def generate_launch_description():

    myrt_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_base_controller",
                    "--controller-manager",
                    "/controller_manager"
        ]
       
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ]
    )

    

    return LaunchDescription(
        [

            joint_state_broadcaster_spawner,
            myrt_controller_spawner,
            
        ]
    )