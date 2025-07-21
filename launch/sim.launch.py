from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.substitution import Substitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

class ConcatenateSubstitutions(Substitution):
    def __init__(self, *substitutions):
        self.substitutions = substitutions

    def perform(self, context):
        return ''.join([sub.perform(context) for sub in self.substitutions])

def generate_launch_description():
    # Get the directories of the involved packages
    vortex_stonefish_sim_dir = get_package_share_directory('rbi_sim')
    stonefish_ros2_dir = get_package_share_directory('stonefish_ros2')

    # Set default paths for simulation_data and scenario_desc
    simulation_data_default = PathJoinSubstitution([vortex_stonefish_sim_dir, 'data'])
    scenario_desc_default = PathJoinSubstitution([vortex_stonefish_sim_dir, 'scenarios'])

    # Declare arguments
    simulation_data_arg = DeclareLaunchArgument(
        'simulation_data',
        default_value=simulation_data_default,
        description='Path to the simulation data folder'
    )

    scenario_desc_arg = DeclareLaunchArgument(
        'scn',
        default_value=PathJoinSubstitution('ocean'),
        description='Path to the scenario file',
        choices=[
            'ocean',
            'testing',
            'plane',
        ]
    )

    window_res_x_arg = DeclareLaunchArgument(
        'window_res_x',
        default_value='1820',
        description='Window resolution width'
    )

    window_res_y_arg = DeclareLaunchArgument(
        'window_res_y',
        default_value='1000',
        description='Window resolution height'
    )

    quality_arg = DeclareLaunchArgument(
        'rendering_quality',
        default_value='high',
    )

    # Logic to prepend the scenarios directory if only a filename is provided
    scenario_desc_resolved = PathJoinSubstitution([
        vortex_stonefish_sim_dir, 
        'scenarios', 
        ConcatenateSubstitutions(LaunchConfiguration('scn'), TextSubstitution(text='.scn'))
    ])

    # Include the original launch file from stonefish_ros2 package
    include_stonefish_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stonefish_ros2_dir, '/launch/stonefish_simulator.launch.py']),
        launch_arguments={
            'simulation_data': LaunchConfiguration('simulation_data'),
            'scenario_desc': scenario_desc_resolved,
            'window_res_x': LaunchConfiguration('window_res_x'),
            'window_res_y': LaunchConfiguration('window_res_y'),
            'rendering_quality': LaunchConfiguration('rendering_quality')
        }.items()
    )

    # Lidar convertion node
    lidar_converter_node = Node(
        package='rbi_sim',  # Replace with actual package name
        executable='LiDAR_converter',  # Name of your built executable
        name='lidar_converter',        # Optional: give the node a unique name
        output='screen',               # Optional: log output to screen
        parameters=[{
        }]
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[
            # adjust these if you like:
            {'deadzone': 0.05},         # ignore small stick deflections
            {'autorepeat_rate': 20.0},  # Hz
        ],
    )

    thruster_node = Node(
        package='rbi_sim',
        executable='thrust_allocator',
        name='thruster_allocator',
        output='screen',
        parameters=[
            {'throttle_axis': 1},
            {'steering_axis': 3},
            {'max_forward_thrust':    350.0},
            {'max_steering_thrust':    75.0},
        ],
    )

    tf_node1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_publisher_zed1',
        output='screen',
        arguments=[
            '--x', '-0.297',
            '--y', '-0.296',
            '--z', '-0.425',
            '--qx', '0.7071',
            '--qy', '0.0',
            '--qz', '0.0',
            '--qw', '0.7071',
            '--frame-id', 'BlueBoat/ZedCam1',
            '--child-frame-id', 'LiDAR',
        ],
    )

    return LaunchDescription([
        simulation_data_arg,
        scenario_desc_arg,
        window_res_x_arg,
        window_res_y_arg,
        quality_arg,
        include_stonefish_launch,
        lidar_converter_node,
        joy_node,
        thruster_node,
        tf_node1,
    ])