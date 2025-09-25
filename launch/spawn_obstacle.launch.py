import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression


def generate_launch_description():
    # Get the urdf file

    model_folder = 'unit_obstacle'
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )

    obstacle_launch_path =  os.path.join(
        get_package_share_directory('cctv_layer_ros2'), 'launch')

    ld = LaunchDescription()
    for human_id in range(0, 1):
        x_pose = '4.0'
        y_pose = PythonExpression([str(human_id)]) 

        start_gazebo_ros_spawner_cmd = Node(
            package='gazebo_ros',
            namespace=f"walking_human{human_id+1}",
            executable='spawn_entity.py',
            arguments=[
                '-entity', f'unit_obstacle_{human_id+1}',
                '-file', urdf_path,
                '-x', x_pose,
                '-y', y_pose,
                '-z', '0.00'
            ],
            output='screen',
            
        )
        ld.add_action(start_gazebo_ros_spawner_cmd)

    obstacle_launch_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([obstacle_launch_path, '/obstacle.launch.py'])
    )

    ld.add_action(obstacle_launch_cmd)


    return ld
