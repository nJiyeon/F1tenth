import os
import pathlib
import yaml 

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def loader_config_from_src(package_name: str, yaml_file: str, parents_dir='ICRA2026-Paper'):
    from pathlib import Path

    # 현재 share 디렉토리 경로 가져오기
    shared_path = get_package_share_directory(package_name)
    p = Path(shared_path)
    
    workspace_root = None
    for i, part in enumerate(p.parts):
        if part == "install":
            workspace_root = Path(*p.parts[:i])
            break
    
    if workspace_root is None:
        raise ValueError(f"Could not find workspace root from path: {shared_path}")
    
    # src 디렉토리의 config 경로 구성
    src_config_path = workspace_root / "src" / parents_dir / yaml_file
    
    if src_config_path.exists():
        print(f"Parameter loaded from `{package_name}` source folder.")
        return os.path.abspath(src_config_path)
    else:
        # fallback
        return os.path.join(
            get_package_share_directory(package_name),
            yaml_file
        )


def generate_launch_description():

    mppi_config = loader_config_from_src('f1tenth_mppi', 'config/mppi.yaml')
    nav_config = loader_config_from_src('f1tenth_mppi', 'config/nav.yaml')
    common_config = loader_config_from_src('f1tenth_mppi', 'config/common.yaml')
    obs_config = loader_config_from_src('f1tenth_mppi', 'config/obs.yaml')
    with open(common_config, 'r') as f:
        common_config_dict = yaml.safe_load(f)

    if common_config_dict['mppi_controller'] == "jax":
        print("Using JAX MPPI Node...")
        mppi_executable = 'jax_mppi'
        mppi_package = 'f1tenth_mppi'
    elif common_config_dict['mppi_controller'] == "cpp":
        print("Using C++ MPPI Node...")
        mppi_executable = 'mppi_controller_node'
        mppi_package = 'mppi_controller_cpp'
    elif common_config_dict['mppi_controller'] == "cpp_integrated":
        print("Using C++ Integrated MPPI Node...")
        mppi_executable = 'mppi_controller_integrated_node'
        mppi_package = 'mppi_controller_cpp'
    else:
        mppi_executable = 'mppi'
        mppi_package = 'f1tenth_mppi'

    ld = LaunchDescription()


    # Nodes 
    mppi_node = Node(
        package=mppi_package,
        executable=mppi_executable,
        name='mppi',
        parameters=[mppi_config, 
            {'waypoint_path': common_config_dict['waypoint_path'],
             'pose_topic': common_config_dict['racecar_odom_topic'],
             'scan_topic': common_config_dict['racecar_scan_topic'],
             'vehicle_frame': common_config_dict['racecar_frame']
         }
        ],
    )
    nav_node = Node(
        package='f1tenth_mppi',
        executable='navigation',
        name='navigation_control',
        parameters=[nav_config, {'waypoint_path': common_config_dict['waypoint_path'],
                                  'odom_topic': common_config_dict['racecar_odom_topic']}],
    )


    start_nav_after_mppi = RegisterEventHandler(
            OnProcessStart(
                target_action=mppi_node,
                on_start=[
                    TimerAction(
                        period=3.0,
                        actions=[nav_node]
                    )
                ]
            )
        )
    

    if not common_config_dict['mppi_controller'] == "cpp_integrated":
        print("Using separate Obstacle Detector Node...")
        obs_node = Node(
            package='f1tenth_mppi',
            executable='obstacle_detector',
            name='obstacle_detector',
            parameters=[obs_config,
                {'vehicle_frame': common_config_dict['gym_ros_frame'],
                'scan_topic': common_config_dict['gym_ros_scan_topic']}
            ]
        )

        
        start_mppi_after_obs = RegisterEventHandler(
            OnProcessStart(
                target_action=obs_node,
                on_start=[
                    TimerAction(
                        period=2.0,
                        actions=[mppi_node]
                    )
                ]
            )
        )
        ld.add_action(obs_node)
        ld.add_action(start_mppi_after_obs)
        ld.add_action(start_nav_after_mppi)
    else:
         # Integrated node handles obstacle detection internally
        ld.add_action(mppi_node)
        ld.add_action(start_nav_after_mppi)


    return ld 