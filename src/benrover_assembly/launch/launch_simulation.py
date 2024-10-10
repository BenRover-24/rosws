from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import xacro
import importlib
import launch
import launch.actions
importlib.reload(launch)
importlib.reload(launch.actions)
import xml.etree.ElementTree as ET
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable, resolve

def generate_launch_description():

    # Déclaration des arguments
    world_arg = DeclareLaunchArgument(
        'world', 
        default_value='mars_exploration_rovers_world', 
        description='Choisir le monde Gazebo à utiliser.'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='benrover_assembly', 
        description='Nom du robot dans la simulation.'
    )

    # Chargement des configurations
    world_name = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')

    # Chemin complet vers le monde Gazebo
    world_path = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'), 
        'worlds', 
        world_name,
        'world.sdf' 
    ])

    # Chargement du fichier URDF
    urdf_file = PathJoinSubstitution([
        FindPackageShare('benrover_assembly'),
        'urdf',
        'benrover_assembly.urdf' 
    ])
    robot_description_config = xacro.process_file(urdf_file).toxml()

    # Définition des nodes ROS2
    def generate_static_tf_publishers(urdf_content):
        tree = ET.fromstring(urdf_content)
        publishers = []
        for joint in tree.iter('joint'):
            parent_link = joint.find('parent').attrib['link']
            child_link = joint.find('child').attrib['link']
            arguments = [
                '0', '0', '0', '0', '0', '0',
                parent_link,
                child_link
            ]
            node = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'static_tf_publisher_{parent_link}_{child_link}',
                arguments=arguments,
                output='screen'
            )
            publishers.append(node)
        return publishers

    # ---  Lancement ---
    ld = LaunchDescription() #  <-- Initialiser ld ici

    # ---  Ajout des actions ---
    ld.add_action(world_arg)
    ld.add_action(robot_name_arg)
    

    # robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )
    ld.add_action(robot_state_publisher) 

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('benrover_assembly'), 'rviz', 'your_config.rviz'])] 
    )
    ld.add_action(rviz_node)

    # Static TF Publishers
    static_tf_publishers = generate_static_tf_publishers(robot_description_config)
    for publisher in static_tf_publishers:
        ld.add_action(publisher)

    # Inclusion du serveur Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
           str(PathJoinSubstitution([
                FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'
            ]))
        ),
        launch_arguments={
            'world': world_path
        }.items()
    )
    ld.add_action(gazebo) 

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robot_name],  # <-- Supprimer -file 
        output='screen'
    )
    ld.add_action(spawn_entity)


    return ld