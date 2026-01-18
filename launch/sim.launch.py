import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "orin_car"

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), "launch", "robot.launch.py")]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    world_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'obstacle.world')
    
	# Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")]
        ),
		launch_arguments={'world': world_path}.items(),
    )

    # [수정 포인트] 로봇을 z=0.5m 높이에서 소환하도록 -z 옵션 추가
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "my_bot", "-z", "0.5"],
        output="screen",
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
    )

    slam_config_path = os.path.join(get_package_share_directory(package_name), 'config', 'slam_params.yaml')

    # SLAM Toolbox를 실행할 때 파라미터 파일을 인자로 넘겨줍니다.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_config_path
        }.items()
    )

    return LaunchDescription(
        [
            rsp,
			joint_state_publisher,
            gazebo,
            spawn_entity,
			slam,
        ]
    )
