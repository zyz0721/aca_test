import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            RegisterEventHandler, ExecuteProcess)

from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node

# 加载机器人描述
def load_robot_description(robot_description_path):
    """
    从Xacro文件加载机器人描述，使用YAML文件中的参数

    @param robot_description_path: 机器人Xacro文件的路径
    @return: 包含机器人URDF XML描述的字符串
    """

    # 处理Xacro文件生成机器人的URDF表示
    robot_description = xacro.process_file(
        robot_description_path)

    return robot_description.toxml()

# 启动车辆控制器
def start_vehicle_control():
    """
    启动车辆在ROS 2中运行所需的各种控制器

    @return: 包含关节状态、前进速度和前进位置控制器的ExecuteProcess动作元组
    """
    # 启动关节状态广播器
    joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'joint_state_broadcaster'],
        output='screen')
    # joint_state_controller = Node(package='controller_manager',executable='spawner',
    #     arguments=['joint_state_broadcaster'], output='screen')
    # 启动底盘控制器
    forward_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'forward_velocity_controller'],
        output='screen')    
    forward_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'forward_position_controller'],
        output='screen')

    # 启动腰部控制器
    waist_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'waist_position_controller'],
        output='screen')

    # 启动头部控制器
    head_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'head_position_controller'],
        output='screen')

    # 启动左臂控制器
    left_arm_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'left_arm_position_controller'],
        output='screen')

    # 启动右臂控制器
    right_arm_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'right_arm_position_controller'],
        output='screen')

    return (joint_state_controller,
            forward_velocity_controller,
            forward_position_controller,
            waist_position_controller,
            head_position_controller,
            left_arm_position_controller,
            right_arm_position_controller)

# launch文件启动主函数
def generate_launch_description():
    # 获得包名和包路径
    package_name = "galbot_s1_v1_description"
    package_path = get_package_share_directory(package_name)

    # 添加模型库目录，不然找不到
    gazebo_models_path = "/home/galbot/gazebo_models"
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    # 指定Gazebo世界文件
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(package_path, 'world', 'maze.sdf'),
        description='SDF world file'
    )

    # 定义初始位姿的启动参数
    x_arg = DeclareLaunchArgument('x', default_value='0.0',
                                  description='Initial X position')

    y_arg = DeclareLaunchArgument('y', default_value='0.0',
                                  description='Initial Y position')

    z_arg = DeclareLaunchArgument('z', default_value='0.1',
                                  description='Initial Z position')

    roll_arg = DeclareLaunchArgument('R', default_value='0.0',
                                     description='Initial Roll')

    pitch_arg = DeclareLaunchArgument('P', default_value='0.0',
                                      description='Initial Pitch')

    yaw_arg = DeclareLaunchArgument('Y', default_value='0.0',
                                    description='Initial Yaw')

    # 获取启动配置
    world_file = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('R')
    pitch = LaunchConfiguration('P')
    yaw = LaunchConfiguration('Y')

    # 设置Xacro模型和配置文件的路径
    robot_description_path = os.path.join(package_path, 'urdf',
                                          'galbot_S1_V1_with_GE10301.urdf.xacro')
    
    gz_bridge_params_path = os.path.join(package_path, 'config',
                                         'ros_gz_bridge.yaml')

    # 加载 URDF 描述
    robot_description = load_robot_description(robot_description_path)

    # gazebo仿真启动文件
    gazebo_pkg_launch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'),
                     'launch',
                     'gz_sim.launch.py'))

    # gazebo启动描述
    gazebo_launch = IncludeLaunchDescription(
        gazebo_pkg_launch,
        launch_arguments={'gz_args': [f'-r -v 4 ', world_file],
                          'on_exit_shutdown': 'true'}.items())

    robot_name = "galbot_s1"
    
    # 创建节点以在gazebo中生成机器人模型
    spawn_model_gazebo_node = Node(package='ros_gz_sim',
                                   executable='create',
                                   arguments=['-name', robot_name,
                                              '-string', robot_description,
                                              '-x', x,
                                              '-y', y,
                                              '-z', z,
                                              '-R', roll,
                                              '-P', pitch,
                                              '-Y', yaw,
                                              '-allow_renaming', 'false'],
                                   output='screen')

    # 创建节点发布机器人状态
    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description,
                                          'use_sim_time': True}],
                                      output='screen')
    
    # 创建ROS-Gazebo桥接节点以处理消息传递
    gz_bridge_node = Node(package='ros_gz_bridge',
                          executable='parameter_bridge',
                          arguments=['--ros-args', '-p',
                                     f'config_file:={gz_bridge_params_path}'],
                          output='screen')

    # 启动控制器
    joint_state, forward_velocity, forward_position, waist_position,head_position,left_arm_position, right_arm_position = start_vehicle_control()

    # 创建启动描述
    launch_description = LaunchDescription([
        # 注册事件处理器：在生成模型节点退出后启动关节状态控制器
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=spawn_model_gazebo_node,
                                        on_exit=[joint_state])),
        # 注册事件处理器：在关节状态控制器启动后启动速度和位置控制器
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=joint_state,
                                        on_exit=[forward_velocity,
                                                 forward_position,
                                                 waist_position,
                                                 head_position,
                                                 left_arm_position,
                                                 right_arm_position])),
                                        world_arg,
                                        gazebo_launch,
                                        x_arg,
                                        y_arg,
                                        z_arg,
                                        roll_arg,
                                        pitch_arg,
                                        yaw_arg,
                                        spawn_model_gazebo_node,
                                        robot_state_publisher_node,
                                        gz_bridge_node])

    return launch_description
