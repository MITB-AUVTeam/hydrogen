import os

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition

def launch_setup(context: LaunchContext, *args, **kwargs):
    
    # 1. Get configuration from arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    rgb_topic_front = LaunchConfiguration('rgb_topic_front')
    depth_topic_front = LaunchConfiguration('depth_topic_front')
    camera_info_topic_front = LaunchConfiguration('camera_info_topic_front')
    rgb_topic_down = LaunchConfiguration('rgb_topic_down')
    depth_topic_down = LaunchConfiguration('depth_topic_down')
    camera_info_topic_down = LaunchConfiguration('camera_info_topic_down')
    frame_id = LaunchConfiguration('frame_id').perform(context)
    
    # 2. Defining parameters 
    base_parameters=[{
        'frame_id': frame_id,
        'subscribe_rgbd': True,
        'subscribe_odom_info': True,
        'approx_sync': False, 
        'use_sim_time': use_sim_time,
        'wait_imu_to_init': False # Disabled as most sims don't sync IMU with standard plugins
    }]
    rtbmap_parameters = [{
        **base_parameters[0],
        'rgbd_cameras':2,
        'Vis/EstimationType': '0', 
    }]    

    return [
        # Node 1: Sync rgb/depth/camera_info together
        # This creates an 'rgbd_image' topic that RTAB-Map needs
        Node(   
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            name='rgbd_sync_front',
            parameters=base_parameters,
            remappings=[
                ('rgb/image', rgb_topic_front),
                ('rgb/camera_info', camera_info_topic_front),
                ('depth/image', depth_topic_front),
                ('rgbd_image', '/rgbd_image0'), # Output topic for RTAB-Map
            ]   
        ),
        Node(   
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            name='rgbd_sync_down',
            parameters=base_parameters,
            remappings=[
                ('rgb/image', rgb_topic_down),
                ('rgb/camera_info', camera_info_topic_down),
                ('depth/image', depth_topic_down),
                ('rgbd_image', '/rgbd_image1'), # Output topic for RTAB-  Map
            ],   
        ),
        # Node 2: Visual Odometry
        # Computes odometry from the camera images (since we aren't using ZED's internal odom)
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters= base_parameters,
            remappings=[
                ('rgbd_image','/rgbd_image0'),
            ],
            arguments=['--ros-args', '--log-level', 'warn'] # Reduce noise
        ),
        # Node 3: RTAB-Map SLAM
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=rtbmap_parameters,
            remappings=[
                ('rgbd_image','/rgbd_image0'),
                ('rgbd_image2','/rgbd_image1'),
            ],
            arguments=['-d'] # Delete previous database on start
        ),

        # Node 4: Visualization
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=rtbmap_parameters,
            remappings=[
                ('rgbd_image',  '/rgbd_image0'),
                ('rgbd_image2', '/rgbd_image1'),
            ],
        ),

        Node(
            package='control_system', executable='global_planner', output='screen', 
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        
        # Simulation time is crucial for Gazebo/Ignition
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # Topic Arguments (Change default_value to match your sim if you want)
        DeclareLaunchArgument(
            'rgb_topic_front', default_value='/camera/RGB_image_raw/front',
            description='Topic for the raw RGB image,front camera'),
            
        DeclareLaunchArgument(
            'depth_topic_front', default_value='/camera/depth_image_raw/front',
            description='Topic for the depth image,front camera'),

        DeclareLaunchArgument(
            'camera_info_topic_front', default_value='/camera_info_front',
            description='Topic for the camera info,front camera'),

        DeclareLaunchArgument(
            'frame_id', default_value='base_link',
            description='The TF frame of the camera (Optical frame, Z-forward)'),
        DeclareLaunchArgument(
            'rgb_topic_down', default_value='/camera/RGB_image_raw/down',
            description='Topic for the raw RGB image,down camera'),
        DeclareLaunchArgument(
            'depth_topic_down', default_value='/camera/depth_image_raw/down',
            description='Topic for the depth image,down camera'),
        DeclareLaunchArgument(
            'camera_info_topic_down', default_value='/camera_info_down',
            description='Topic for the camera info,down camera'),

        OpaqueFunction(function=launch_setup)
    ])