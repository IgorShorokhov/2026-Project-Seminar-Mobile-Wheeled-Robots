from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    TimerAction,
    LogInfo,
    RegisterEventHandler,
)
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart, OnProcessExit


def generate_launch_description():

    turtlesim = Node(
        package="turtlesim", executable="turtlesim_node", name="turtlesim"
    )

    kill_turtle = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/kill",
            "turtlesim/srv/Kill",
            "{name: turtle1}",
        ],
        output="screen",
    )

    spawn_left_turtle = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/spawn",
            "turtlesim/srv/Spawn",
            "{x: 2, y: 2, theta: 0, name: 'turtle_left'}",
        ],
        output="screen",
    )
    spawn_right_turtle = ExecuteProcess(
        cmd=[
              "ros2",
             "service",
              "call",
              "/spawn",
             "turtlesim/srv/Spawn",
               "{x: 6, y: 2, theta: 0, name: 'turtle_right'}",
        ],
        output="screen",
    )


    left_turtle_controller = Node(
        package="practice_01_turtlesim",
        executable="digit_drawer",
        name="left_turtle_controller",
        parameters=[{'turtle': 'turtle_left', 'digit': 1}],
        output = 'screen'
    )
    right_turtle_controller = Node(
            package="practice_01_turtlesim",
            executable="digit_drawer",
            name="right_turtle_controller",
            parameters=[{'turtle': 'turtle_right', 'digit': 8}],
            output = 'screen'
        )
    kill_default_turtle = TimerAction(
        period=1.0,
        actions=[kill_turtle],
    )

    run_left_turtle = RegisterEventHandler(OnProcessExit(
        target_action=kill_turtle,
        on_exit=[spawn_left_turtle]
    ))
    run_right_turtle = RegisterEventHandler(OnProcessExit(
            target_action=spawn_left_turtle,
            on_exit=[spawn_right_turtle]
        ))
    run_controllers_turtle = RegisterEventHandler(OnProcessExit(
                target_action=spawn_right_turtle,
                on_exit=[left_turtle_controller, right_turtle_controller]
            ))
    return LaunchDescription(
        [
            turtlesim,
            kill_default_turtle, 
            run_left_turtle,
            run_right_turtle,
            run_controllers_turtle
        ]
    )