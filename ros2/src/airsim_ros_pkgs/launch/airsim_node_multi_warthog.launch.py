import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    num_vehicles = int(LaunchConfiguration('num_vehicles').perform(context))
    veh_name = LaunchConfiguration('veh_name').perform(context)

    remappings = []
    nodes = []

    for i in range(1, num_vehicles+1):
        suffix = f"{i}" if i > 1 else ""
        vehicle = f"{veh_name}{suffix}"
        print(vehicle)

        remap = [
            (f'/airsim_node_warthog/{vehicle}/gps/gps', f'/{vehicle}/gps'),
            (f'/airsim_node_warthog/{vehicle}/warthog_cmd', f'/{vehicle}/cmd_vel'),
            (f'/airsim_node_warthog/{vehicle}/global_gps', f'/{vehicle}/gps1'),
            (f'/airsim_node_warthog/{vehicle}/altimeter/barometer', f'/{vehicle}/barometer'),
            (f'/airsim_node_warthog/{vehicle}/environment', f'/{vehicle}/environment'),
            (f'/airsim_node_warthog/{vehicle}/imu/imu', f'/{vehicle}/imu/data'),
            (f'/airsim_node_warthog/{vehicle}/base_link', f'/{vehicle}/platform/odom'),
            (f'/airsim_node_warthog/{vehicle}/warthog_state', f'/{vehicle}/warthog_state'),
            (f'/airsim_node_warthog/{vehicle}/lidar/lidar_link', f'/{vehicle}/lidar_points'),
            (f'/airsim_node_warthog/{vehicle}/camera_2/Scene', f'/{vehicle}/multisense_front/color/image_raw'),
            (f'/airsim_node_warthog/{vehicle}/camera_2/Scene/camera_info', f'/{vehicle}/multisense_front/color/camera_info'),
            (f'/airsim_node_warthog/{vehicle}/camera_1/DepthPlanar', f'/{vehicle}/multisense_front/depth/image_rect_raw'),
            (f'/airsim_node_warthog/{vehicle}/camera_1/DepthPlanar/camera_info', f'/{vehicle}/multisense_front/depth/camera_info'),
        ]

        node = Node(
            package='airsim_ros_pkgs',
            executable='airsim_node_warthog',
            name='airsim_node_warthog',
            output='screen',
            remappings=remap,
            parameters=[{
                'is_vulkan': False,
                'update_airsim_img_response_every_n_sec': 0.2,
                'update_airsim_control_every_n_sec': 0.001,
                'update_lidar_every_n_sec': 0.1,
                'publish_clock': LaunchConfiguration('publish_clock'),
                'host_ip': LaunchConfiguration('host')
            }]
        )
        nodes.append(node)

    tf_publisher = Node(
        package='airsim_ros_pkgs',
        executable='tf_publisher',
        name='tf_publisher',
        output='screen',
        #parameters=[
        #    {'veh_name': veh_name, 'num_vehicles': num_vehicles}
        #]
    )

    nodes.append(tf_publisher)
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output', default_value='log'),
        DeclareLaunchArgument('publish_clock', default_value='False'),
        DeclareLaunchArgument('is_vulkan', default_value='True'),
        DeclareLaunchArgument('host', default_value='localhost'),
        DeclareLaunchArgument('veh_name', default_value='warty'),
        DeclareLaunchArgument('num_vehicles', default_value='2'),
        OpaqueFunction(function=launch_setup),
    ])

