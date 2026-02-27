import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Arguments
    # Use this file's location to get config files
    launch_file_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(launch_file_dir)
    config_dir = os.path.join(workspace_dir, 'config')

    # Parameters from environment variables (sandbox defaults)
    env_params = {
        "twin_url":                os.getenv("MUTO_TWIN_URL", "https://ditto:ditto@sandbox.composiv.ai"),
        "twin_user":               os.getenv("MUTO_TWIN_USER", "ditto"),
        "twin_password":           os.getenv("MUTO_TWIN_PASSWORD", "ditto"),
        "host":                    os.getenv("MUTO_HOST", "sandbox.composiv.ai"),
        "port":                    int(os.getenv("MUTO_PORT", "1883")),
        "symphony_user":           os.getenv("MUTO_SYMPHONY_USER", "admin"),
        "symphony_password":       os.getenv("MUTO_SYMPHONY_PASSWORD", ""),
        "symphony_host":           os.getenv("MUTO_SYMPHONY_HOST", "localhost"),
        "symphony_port":           int(os.getenv("MUTO_SYMPHONY_PORT", "1883")),
        "symphony_api_url":        os.getenv("MUTO_SYMPHONY_API_URL", "http://localhost:8082/v1alpha2/"),
        "symphony_broker_address": os.getenv("MUTO_SYMPHONY_BROKER_ADDRESS", "tcp://mosquitto:1883"),
    }

    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'enable_symphony',
            default_value='true',
            description='Enable Symphony MQTT provider',
            choices=['true', 'false']
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='INFO',
            description='Logging level for all nodes',
            choices=['DEBUG', 'INFO', 'WARN', 'ERROR']
        ),

        DeclareLaunchArgument(
            'muto_config_file',
            default_value= os.path.join(config_dir, "muto.yaml"),
            description='Path to global Muto configuration file'
        ),
        DeclareLaunchArgument("muto_namespace", default_value="muto"),
        DeclareLaunchArgument(
            "vehicle_namespace",
            default_value="org.eclipse.muto.sandbox",
            description="Vehicle ID namespace",
        ),
        DeclareLaunchArgument(
            "vehicle_name",  description="Vehicle ID"
        )
    ]
    
    # Configuration parameters
    enable_symphony = LaunchConfiguration('enable_symphony')
    log_level = LaunchConfiguration('log_level')
    muto_config_file = LaunchConfiguration('muto_config_file')
    vehicle_namespace = LaunchConfiguration('vehicle_namespace')
    vehicle_name = LaunchConfiguration('vehicle_name')
    muto_namespace = LaunchConfiguration('muto_namespace')


    # Agent
    node_agent = Node(
        namespace=muto_namespace,
        name="agent",
        package="muto_agent",
        executable="muto_agent",
        output="screen",
        parameters=[
            muto_config_file,
            {"namespace": vehicle_namespace},
            {"name": vehicle_name},
            env_params,
        ],
    )

    node_mqtt_gateway = Node(
        namespace=muto_namespace,
        name="gateway",
        package="muto_agent",
        executable="mqtt",
        output="screen",
        parameters=[
            muto_config_file,
            {"namespace": vehicle_namespace},
            {"name": vehicle_name},
            env_params,
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    node_commands = Node(
        namespace=muto_namespace,
        name="commands_plugin",
        package="muto_agent",
        executable="commands",
        output="screen",
        parameters=[
            muto_config_file,
            {"namespace": vehicle_namespace},
            {"name": vehicle_name},
            env_params,
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    # Core
    node_twin = Node(
        namespace=muto_namespace,
        name="core_twin",
        package="muto_core",
        executable="twin",
        output="screen",
        parameters=[
            muto_config_file,
            {"namespace": vehicle_namespace},
            {"name": vehicle_name},
            env_params,
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    # Composer
    node_composer = Node(
        namespace=muto_namespace,
        name="muto_composer",
        package="muto_composer",
        executable="muto_composer",
        output="screen",
        parameters=[
            muto_config_file,
            {"namespace": vehicle_namespace},
            {"name": vehicle_name},
            env_params,
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    node_compose_plugin = Node(
        namespace=muto_namespace,
        name="compose_plugin",
        package="muto_composer",
        executable="compose_plugin",
        output="screen",
        parameters=[
            muto_config_file,
            {"namespace": vehicle_namespace},
            {"name": vehicle_name},
            env_params,
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    node_provision_plugin = Node(
        namespace=muto_namespace,
        name="provision_plugin",
        package="muto_composer",
        executable="provision_plugin",
        output="screen",
        parameters=[
            muto_config_file,
            {"namespace": vehicle_namespace},
            {"name": vehicle_name},
            env_params,
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    node_launch_plugin = Node(
        namespace=muto_namespace,
        name="launch_plugin",
        package="muto_composer",
        executable="launch_plugin",
        output="screen",
        parameters=[
            muto_config_file,
            {"namespace": vehicle_namespace},
            {"name": vehicle_name},
            env_params,
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    symphony_provider = Node(
        namespace=muto_namespace,
        package='muto_agent',
        executable='symphony_provider',
        name='muto_symphony_provider',
        output='screen',
        condition=IfCondition(enable_symphony),
        parameters=[
            muto_config_file,
            {"name": vehicle_name},
            {"namespace": vehicle_namespace},
            {"symphony_target_name": vehicle_name},
            {"symphony_name": vehicle_name},
            env_params,
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    # Launch Description Object
    ld = LaunchDescription()

    # add all declared arguments
    for arg in declared_arguments:
        ld.add_action(arg)

    # add all nodes
    ld.add_action(node_agent)
    ld.add_action(node_mqtt_gateway)
    ld.add_action(node_commands)
    ld.add_action(node_twin)
    ld.add_action(node_composer)
    ld.add_action(node_compose_plugin)
    ld.add_action(node_provision_plugin)
    ld.add_action(node_launch_plugin)
    ld.add_action(symphony_provider)

    return ld
