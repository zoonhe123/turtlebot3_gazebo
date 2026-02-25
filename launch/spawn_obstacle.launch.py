import os
import yaml
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()
    
    # ---------------------------------------------------------
    # [설정] 패키지 이름 및 파일 경로
    # ---------------------------------------------------------
    pkg_name = 'cctv_layer_ros2' # 본인 패키지명 확인
    pkg_share = get_package_share_directory(pkg_name)
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    
    # 1. YAML 파일 경로
    config_path = os.path.join(pkg_share, 'config', 'obstacles.yaml')

    # 2. World 파일 경로 (Human Actor 파싱용) - 본인 월드 파일 이름으로 수정 필수!
    world_file_name = 'warehouse.world' 
    world_path = os.path.join(turtlebot3_gazebo_share, 'worlds', world_file_name)

    # ---------------------------------------------------------
    # [로직 1] YAML 파일 읽어서 기계(AMR, Forklift 등) 스폰
    # ---------------------------------------------------------
    if os.path.exists(config_path):
        with open(config_path, 'r') as f:
            config_data = yaml.safe_load(f)
            obstacle_list = config_data.get('obstacles', [])
            
        print(f"[INFO] Loaded {len(obstacle_list)} mechanical obstacles from YAML.")

        for obs in obstacle_list:
            name = obs['name']       # 예: AMR_1
            model_type = obs['type'] # 예: AMR
            waypoints = obs['waypoints'] # [[x,y,yaw,speed], ...]

            # 시작 위치는 첫 번째 웨이포인트
            start_x = str(waypoints[0][0])
            start_y = str(waypoints[0][1])
            start_yaw = str(waypoints[0][2])

            model_sdf_path = os.path.join(turtlebot3_gazebo_share, 'models', model_type, 'model.sdf')

            node = Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name=f'spawn_{name}',
                arguments=[
                    '-entity', f'{name}_obstacle',
                    '-file', model_sdf_path,
                    '-x', start_x, '-y', start_y, '-z', '0.05',
                    '-Y', start_yaw
                ],
                output='screen'
            )
            ld.add_action(node)
    else:
        print(f"[ERROR] YAML file not found: {config_path}")

    # ---------------------------------------------------------
    # [로직 2] World 파일 파싱해서 Human Actor 스폰
    # ---------------------------------------------------------
    if os.path.exists(world_path):
        tree = ET.parse(world_path)
        root = tree.getroot()
        actors = root.findall('.//actor') # 모든 actor 태그 찾기
        
        print(f"[INFO] Found {len(actors)} actors in World file.")
        
        # Human 모델 경로 (가정: turtlebot3_gazebo/models/human/model.sdf)
        human_sdf_path = os.path.join(turtlebot3_gazebo_share, 'models', 'human', 'model.sdf')

        spawn_y_offset = 0.0 # 초기 겹침 방지용 오프셋

        for actor in actors:
            actor_name = actor.attrib['name'] # 예: human_1, human_2
            
            # 이름이 'human'으로 시작하는 경우만 처리
            if actor_name.startswith('human'):
                print(f"[INFO] Spawning obstacle for actor: {actor_name}")
                
                node = Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name=f'spawn_{actor_name}',
                    arguments=[
                        '-entity', f'{actor_name}_obstacle',
                        '-file', human_sdf_path,
                        '-x', '0.0', # Actor 위치는 move_obstacle에서 잡으므로 초기값은 대충
                        '-y', str(spawn_y_offset), 
                        '-z', '0.05'
                    ],
                    output='screen'
                )
                ld.add_action(node)
                spawn_y_offset += 1.0 # 다음 스폰 위치 살짝 띄우기
    else:
        print(f"[WARN] World file not found at {world_path}. Skipping human spawn.")

    # ---------------------------------------------------------
    # [로직 3] Move Obstacle 노드 실행 (통합 제어)
    # ---------------------------------------------------------
    move_node = Node(
        package=pkg_name,
        executable='move_obstacle',
        name='move_obstacle',
        parameters=[{'config_path': config_path}],
        output='screen'
    )

    pub_node = Node(
        package=pkg_name,
        executable='pub_detection',
        name='pub_detection',
        output='screen'
    )


    ld.add_action(move_node)
    ld.add_action(pub_node)

    return ld