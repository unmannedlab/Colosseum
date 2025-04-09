import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    output = DeclareLaunchArgument(
        "output",
        default_value='log')

    publish_clock = DeclareLaunchArgument(
        "publish_clock",
        default_value='False')

    is_vulkan = DeclareLaunchArgument(
        "is_vulkan",
        default_value='True')

    host = DeclareLaunchArgument(
        "host",
        default_value='localhost')
  
    airsim_node = Node(
            package='airsim_ros_pkgs',
            executable='airsim_node_warthog',
            name='airsim_node_warthog',
            output='screen',
            remappings=[
                ('/airsim_node_warthog/warty/gps/gps', '/warty/gps'),
                ('/airsim_node_warthog/warty/warthog_cmd', '/warty/cmd_vel'),
                ('/airsim_node_warthog/warty/global_gps','/warty/gps1'),
                ('/airsim_node_warthog/warty/altimeter/barometer','/warty/barometer'),
	    	    ('/airsim_node_warthog/warty/environment','/warty/environment'),
            	('/airsim_node_warthog/warty/imu/imu','/warty/imu/data'),
	   	        ('/airsim_node_warthog/warty/base_link', '/warty/platform/odom'),
	    	    ('/airsim_node_warthog/warty/warthog_state', '/warty/warthog_state'),
	    	    ('/airsim_node_warthog/gimbal_angle_euler_cmd', '/warty/gimbal_angle_euler_cmd'),
	    	    ('/airsim_node_warthog/gimbal_angle_quat_cmd', '/warty/gimbal_angle_quat_cmd'),
	    	    ('/airsim_node_warthog/origin_geo_point', '/warty/origin_geo_point'),
	    	    ('/airsim_node_warthog/warty/lidar/lidar_link', '/warty/lidar_points'),
                ('/airsim_node_warthog/warty/camera_2/Scene','/warty/multisense_front/color/image_raw'),
                ('/airsim_node_warthog/warty/camera_1/DepthPlanar/camera_info','/warty/multisense_front/depth/camera_info'),
                ('/airsim_node_warthog/warty/camera_1/DepthPlanar','/warty/multisense_front/depth/image_rect_raw'),
                ('/airsim_node_warthog/warty/camera_2/Scene/camera_info','/warty/multisense_front/color/camera_info'),
                ('/airsim_node_warthog/warty2/gps/gps', '/warty2/gps'),
                ('/airsim_node_warthog/warty2/warthog_cmd', '/warty2/cmd_vel'),
                ('/airsim_node_warthog/warty2/global_gps','/warty2/gps1'),
                ('/airsim_node_warthog/warty2/altimeter/barometer','/warty2/barometer'),
	    	    ('/airsim_node_warthog/warty2/environment','/warty2/environment'),
            	('/airsim_node_warthog/warty2/imu/imu','/warty2/imu/data'),
	   	        ('/airsim_node_warthog/warty2/base_link', '/warty2/platform/odom'),
	    	    ('/airsim_node_warthog/warty2/warthog_state', '/warty2/warthog_state'),
	    	    ('/airsim_node_warthog/gimbal_angle_euler_cmd', '/warty2/gimbal_angle_euler_cmd'),
	    	    ('/airsim_node_warthog/gimbal_angle_quat_cmd', '/warty2/gimbal_angle_quat_cmd'),
	    	    ('/airsim_node_warthog/origin_geo_point', '/warty2/origin_geo_point'),
	    	    ('/airsim_node_warthog/warty2/lidar/lidar_link', '/warty2/lidar_points'),
                ('/airsim_node_warthog/warty2/camera_2/Scene','/warty2/multisense_front/color/image_raw'),
                ('/airsim_node_warthog/warty2/camera_1/DepthPlanar/camera_info','/warty2/multisense_front/depth/camera_info'),
                ('/airsim_node_warthog/warty2/camera_1/DepthPlanar','/warty2/multisense_front/depth/image_rect_raw'),
                ('/airsim_node_warthog/warty2/camera_2/Scene/camera_info','/warty2/multisense_front/color/camera_info')
            	],
            parameters=[{
                'is_vulkan': False,
                'update_airsim_img_response_every_n_sec': 0.2,
                'update_airsim_control_every_n_sec': 0.001,
                'update_lidar_every_n_sec': 0.1,
                'publish_clock': LaunchConfiguration('publish_clock'),
                'host_ip': LaunchConfiguration('host')
            }])

    #static_transforms = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(get_package_share_directory('airsim_ros_pkgs'), 'launch/static_transforms.launch.py')
    #    )
    #)
    
    tf_publisher = Node(
        package='airsim_ros_pkgs',
        executable='tf_publisher',
        name='tf_publisher',
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(output)
    ld.add_action(publish_clock)
    ld.add_action(is_vulkan)
    ld.add_action(host)
    ld.add_action(tf_publisher)
    #ld.add_action(static_transforms)
    ld.add_action(airsim_node)

    return ld
