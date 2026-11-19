import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Twist
from turtlesim.msg import Pose as TurtlePose

digits = {1: [
    ("turn", math.pi / 2),
    ("move", 6),
    ("turn", math.pi * (1 + 1 / 4)),
    ("move", math.hypot(1, 1))
    ],
    8: [
    ("move", 3), 
    ("turn", math.pi / 2),
    ("move", 6),
    ("turn", math.pi),
    ("move", 3),
    ("turn", math.pi *(1 + 1/ 2)),
    ("move", 3),
    ("turn", 0),
    ("move", 3),
    ("turn", math.pi), 
    ("move", 3),
    ("turn", math.pi * 3 / 2),
    ("move", 3)
    ]}
def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class DigitDrawer(Node):

    def __init__(self):
        super().__init__("DigitDrawer")
        self.declare_parameter("linear_speed", 10)
        self.declare_parameter("angular_speed", 1)
        self.declare_parameter("control_hz", 1000)
        self.declare_parameter("turtle", "turtle1")
        self.declare_parameter("digit", 1)

        self.turtle = str(self.get_parameter("turtle").value)
        self.digit = int(self.get_parameter("digit").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.control_hz = float(self.get_parameter("control_hz").value)

        pose_topic = f"/{self.turtle}/pose"
        cmd_topic = f"/{self.turtle}/cmd_vel"

        self.movements = digits[self.digit]

        self.pose_sub = self.create_subscription(TurtlePose, pose_topic, self.on_pose, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        dt = 1 / self.control_hz

        self.timer = self.create_timer(dt, self.on_timer)

        self.pose = None
        self.action_index = 0
        self.was_drawwed = False
        self.segment_start = None

        self.get_logger().info(
            f"Drawing digit {self.digit} with turtle {self.turtle}"
        )


        self.angle_pred = 0.02
        self.dist_pred = 0.05
    def on_pose(self, msg: TurtlePose):
        self.pose = msg

    def publish_none(self):
        self.cmd_pub.publish(Twist())


    def finish_action(self):
        self.publish_none()
        self.action_index += 1
        self.segment_start = None

    def on_timer(self):
        if self.pose is None:
            self.publish_none()
            return
        if self.action_index >= len(self.movements):
            self.publish_none()
            if not self.was_drawwed:
                self.get_logger().info(
                    f"digit {self.digit} was drawed"
                )
                self.was_drawwed = True

            return
            

        action, target = self.movements[self.action_index]
        command = Twist()
        if action == "turn":
            theta = self.pose.theta

            angle_error = wrap_to_pi(target - theta)

            if angle_error > self.angle_pred:
                command.angular.z = self.angular_speed
            elif angle_error < - self.angle_pred:
                command.angular.z = -self.angular_speed
            else:
                self.finish_action()
                return

        elif action == "move":
            if self.segment_start is None:
                self.segment_start = (self.pose.x, self.pose.y)
            start_x, start_y = self.segment_start

            current_x, current_y = self.pose.x, self.pose.y

            current_dist = math.hypot(current_x - start_x, current_y - start_y)

            if current_dist > target - self.dist_pred:
                self.finish_action()
                return
            command.linear.x = self.linear_speed

        self.cmd_pub.publish(command)


def main(args = None):
    rclpy.init(args = args)
    node = DigitDrawer()
    try:
        rclpy.spin(node)
    finally:
        node.publish_none()
        node.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()