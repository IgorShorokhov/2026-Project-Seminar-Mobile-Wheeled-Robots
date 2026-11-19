import math
import os

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformException, TransformListener


class ConvergenceLogger(Node):
    def __init__(self):
        super().__init__("convergence_logger")
        self.sim_time = None
        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.gt = None
        self.amcl = None
        self.declare_parameter("position_threshold", 0.2)
        self.declare_parameter("yaw_threshold", 0.2)
        self.declare_parameter("hold_duration", 5.0)
        self.position_threshold = self.get_parameter("position_threshold").value
        self.yaw_threshold = self.get_parameter("yaw_threshold").value
        self.hold_duration = self.get_parameter("hold_duration").value

        self.declare_parameter("duration", 60.0)
        self.declare_parameter("output_dir", "results")
        self.declare_parameter("run_name", "test")
        self.duration = self.get_parameter("duration").value
        self.output_dir = self.get_parameter("output_dir").value
        self.run_name = self.get_parameter("run_name").value

        self.times = []
        self.gt_x = []
        self.gt_y = []
        self.amcl_x = []
        self.amcl_y = []
        self.errors_position = []
        self.errors_yaw = []
        self.done = False
        self.start_time = None
        self.convergence_time = None
        self.interval_start = None

    def on_clock(self, message):
        self.sim_time = message.clock.sec + message.clock.nanosec * 1e-9
        self.gt = self.get_pose("base_link_gt", Time())
        if self.gt is None:
            return
        self.amcl = self.get_pose("base_link", Time(seconds=self.gt[0]))

        if self.amcl is None:
            return
        if self.start_time is None:
            self.start_time = self.gt[0]

        dx = self.gt[1] - self.amcl[1]
        dy = self.gt[2] - self.amcl[2]

        error_position = math.hypot(dx, dy)

        error_yaw = abs(self.wrap(self.gt[3] - self.amcl[3]))

        self.get_logger().info(
            f"t={self.sim_time:.1f} e_p={error_position:.3f} e_yaw={error_yaw:.3f}",
            throttle_duration_sec=1.0,
        )

        if error_position < self.position_threshold and self.yaw_threshold > error_yaw:
            if self.interval_start is None:
                self.interval_start = self.gt[0]
            else:
                if (
                    self.convergence_time is None
                    and self.gt[0] - self.interval_start >= self.hold_duration
                ):
                    self.convergence_time = self.interval_start - self.start_time
                    self.get_logger().info(
                        f"converged: T={self.convergence_time:.1f} s"
                    )
        else:
            self.interval_start = None

        self.times.append(self.gt[0] - self.start_time)
        self.gt_x.append(self.gt[1])
        self.gt_y.append(self.gt[2])
        self.amcl_x.append(self.amcl[1])
        self.amcl_y.append(self.amcl[2])
        self.errors_position.append(error_position)
        self.errors_yaw.append(error_yaw)

        if self.gt[0] - self.start_time >= self.duration:
            self.done = True

    def plot(self):
        fig, ax = plt.subplots(1, 3, figsize=(18, 5.5))

        ax[0].plot(self.gt_x, self.gt_y, label="ground truth")
        ax[0].plot(self.amcl_x, self.amcl_y, label="AMCL")
        ax[0].set_title("Trajectories")
        ax[0].set_xlabel("x, m")
        ax[0].set_ylabel("y, m")
        ax[0].set_aspect("equal")

        ax[1].plot(self.times, self.errors_position, label="position error")
        ax[1].axhline(
            self.position_threshold, linestyle="--", color="red", label="threshold"
        )
        ax[1].set_title("Position error")
        ax[1].set_xlabel("time since start, s")
        ax[1].set_ylabel("error, m")

        ax[2].plot(self.times, self.errors_yaw, label="yaw error")
        ax[2].axhline(
            self.yaw_threshold, linestyle="--", color="red", label="threshold"
        )
        ax[2].set_title("Yaw error")
        ax[2].set_xlabel("time since start, s")
        ax[2].set_ylabel("error, rad")

        if self.convergence_time is None:
            fig.suptitle(f"{self.run_name}: no convergence")
        else:
            fig.suptitle(
                f"{self.run_name}: convergence time {self.convergence_time:.1f} s"
            )
            ax[1].axvline(
                self.convergence_time, linestyle=":", color="green", label="convergence"
            )
            ax[2].axvline(
                self.convergence_time, linestyle=":", color="green", label="convergence"
            )

        for axis in ax:
            axis.grid(True)
            axis.legend()

        os.makedirs(self.output_dir, exist_ok=True)
        fig.savefig(os.path.join(self.output_dir, f"{self.run_name}.png"), dpi=150)
        plt.close(fig)

    def save(self):
        os.makedirs(self.output_dir, exist_ok=True)
        if self.convergence_time is None:
            convergence = np.nan
        else:
            convergence = self.convergence_time
        np.savez(
            os.path.join(self.output_dir, f"{self.run_name}.npz"),
            time=np.array(self.times),
            gt_x=np.array(self.gt_x),
            gt_y=np.array(self.gt_y),
            amcl_x=np.array(self.amcl_x),
            amcl_y=np.array(self.amcl_y),
            errors_position=np.array(self.errors_position),
            errors_yaw=np.array(self.errors_yaw),
            convergence_time=convergence,
            position_threshold=self.position_threshold,
            yaw_threshold=self.yaw_threshold,
        )

    def get_pose(self, child, stamp):
        try:
            tf = self.buffer.lookup_transform("map", child, stamp)
        except TransformException:
            return None
        time_s = Time.from_msg(tf.header.stamp).nanoseconds * 1e-9
        p = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        return (time_s, p.x, p.y, yaw)

    def wrap(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = ConvergenceLogger()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.save()
        node.plot()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
