from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # Launch arguments
    ld.add_action(LBRDescriptionMixin.arg_model(default_value="med7"))
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    ld.add_action(DeclareLaunchArgument(
        name="rviz_cfg_pkg",
        default_value="lbr_bringup",
        description="The package containing the RViz configuration file."
    ))
    ld.add_action(DeclareLaunchArgument(
        name="rviz_cfg",
        default_value="config/mock.rviz",
        description="The RViz configuration file relative to rviz_cfg_pkg."
    ))

    # NOTE: Passing a ParameterValue wrapping a Command substitution is the standard ROS 2 approach.
    # The linter may warn about type mismatch (expects str), but this is correct and works at runtime.
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                PathJoinSubstitution([
                    FindExecutable(name="xacro")
                ]),
                " ",
                PathJoinSubstitution([
                    FindPackageShare("da_vinci_tool_integration"),
                    "urdf",
                    "med7.xacro"
                ]),
                # Add any xacro arguments here if needed
                # e.g. " tool:=sca parent_link:=lbr_link_ee"
            ]),
            value_type=str
        )
    }

    # Robot state publisher using the proper method with namespace
    robot_state_publisher = LBRROS2ControlMixin.node_robot_state_publisher(
        robot_description=robot_description, use_sim_time=False
    )
    ld.add_action(robot_state_publisher)

    # Joint state publisher GUI (for control sliders) with remappings
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        remappings=[
            ("robot_description", "/lbr/robot_description"),
            ("joint_states", "/lbr/joint_states"),
        ]
    )
    ld.add_action(joint_state_publisher_gui)

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([
                FindPackageShare(LaunchConfiguration("rviz_cfg_pkg")),
                LaunchConfiguration("rviz_cfg")
            ])
        ]
    )
    ld.add_action(rviz_node)

    return ld 