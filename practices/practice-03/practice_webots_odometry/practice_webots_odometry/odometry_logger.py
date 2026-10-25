import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TwistStamped
from tf2_ros import Buffer, TransformListener, TransformException
from rosgraph_msgs.msg import Clock

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import os
import math


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class OdometryLogger(Node):

    def __init__(self):
        super().__init__("odometry_logger")
        self.declare_parameter("duration", 130.0)
        self.declare_parameter("output_dir", "results")
        self.declare_parameter("run_name", "noise_0")

        self.duration = float(self.get_parameter("duration").value)
        self.output_dir = self.get_parameter("output_dir").value
        self.run_name = self.get_parameter("run_name").value

        self.buffer = Buffer()
        self.transform_listener = TransformListener(self.buffer, self)

        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        self.create_subscription(TwistStamped, "/cmd_vel", self.on_cmd, 10)

        self.sim_time = None
        self.cmd = None
        self.t0 = None
        self.t_prev = None
        self.gt = []
        self.enc = []
        self.cmd_rows = []

        self.done = False
        self.x = self.y = self.theta = 0.0
        self.get_logger().info("Жду TF и /cmd_vel")

    def get_pose(self, child):
        try:
            tf = self.buffer.lookup_transform("map", child, Time())
        except TransformException:
            return None
        t = Time.from_msg(tf.header.stamp).nanoseconds * 1e-9
        p, q = tf.transform.translation, tf.transform.rotation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        return (t, p.x, p.y, yaw)

    def on_clock(self, message: Clock):
        if self.done:
            return
        now = Time.from_msg(message.clock).nanoseconds * (10**-9)
        self.sim_time = now

        gt = self.get_pose("base_link_gt")
        enc = self.get_pose("base_link")

        if self.t0 is None:
            if gt is not None and enc is not None and self.cmd is not None:
                self.t0 = self.t_prev = now
                _, self.x, self.y, self.theta = gt
                self.get_logger().info(
                    f"Старт записи t0={now:.2f} с, поза GT "
                    f"({self.x:.3f}, {self.y:.3f}, {self.theta:.3f})"
                )

            return
        self.integrate(now)
        if gt is not None:
            self.gt.append(gt)
        if enc is not None:
            self.enc.append(enc)

        self.cmd_rows.append((now, self.x, self.y, self.theta))

        if now - self.t0 >= self.duration:
            self.done = True
            self.finish()

    def on_cmd(self, message: TwistStamped):
        if self.sim_time is not None and self.t0 is not None:
            self.integrate(self.sim_time)
        self.cmd = (message.twist.linear.x, message.twist.angular.z)

    def integrate(self, sim_time: float):
        delta = sim_time - self.t_prev

        if delta <= 0:
            return

        self.t_prev = sim_time
        v = self.cmd[0]
        w = self.cmd[1]
        theta = self.theta

        if abs(w) > 1e-9:
            self.x += v / w * (math.sin(theta + w * delta) - math.sin(theta))
            self.y -= v / w * (math.cos(theta + w * delta) - math.cos(theta))
        else:
            self.x += v * math.cos(theta) * delta
            self.y += v * math.sin(theta) * delta
        self.theta += w * delta

    @staticmethod
    def clean(a: list):
        a_new = np.array(a, dtype=float)
        _, index = np.unique(a_new[:, 0], return_index=True)
        return a_new[index]

    def finish(self):
        gt_np = self.clean(self.gt)
        enc_np = self.clean(self.enc)
        cmd_rows_np = self.clean(self.cmd_rows)
        t = gt_np[:, 0]

        def resample(a):
            return (
                np.interp(t, a[:, 0], a[:, 1]),
                np.interp(t, a[:, 0], a[:, 2]),
                np.interp(t, a[:, 0], np.unwrap(a[:, 3])),
            )

        ex, ey, eth = resample(enc_np)
        cx, cy, cth = resample(cmd_rows_np)

        pos_err_enc = np.hypot(ex - gt_np[:, 1], ey - gt_np[:, 2])
        pos_err_cmd = np.hypot(cx - gt_np[:, 1], cy - gt_np[:, 2])
        yaw_err_enc = np.abs(wrap(eth - gt_np[:, 3]))
        yaw_err_cmd = np.abs(wrap(cth - gt_np[:, 3]))
        tt = t - self.t0

        fig, ax = plt.subplots(1, 3, figsize=(18, 5.5))
        ax[0].plot(gt_np[:, 1], gt_np[:, 2], label="GT")
        ax[0].plot(enc_np[:, 1], enc_np[:, 2], label="Одометрия по энкодерам")
        ax[0].plot(
            cmd_rows_np[:, 1], cmd_rows_np[:, 2], "--", label="Одометрия по командам"
        )
        ax[0].set_aspect("equal", adjustable="datalim")
        ax[0].set_xlabel("x, м")
        ax[0].set_ylabel("y, м")
        ax[0].set_title("Траектории")
        ax[1].plot(tt, pos_err_enc, label="Одометрия по энкодерам")
        ax[1].plot(tt, pos_err_cmd, label="Одометрия по командам")
        ax[1].set_xlabel("Время от начала, с")
        ax[1].set_ylabel("Ошибка положения, м")
        ax[1].set_title("Ошибка положения")
        ax[2].plot(tt, yaw_err_enc, label="Одометрия по энкодерам")
        ax[2].plot(tt, yaw_err_cmd, label="Одометрия по командам")
        ax[2].set_xlabel("Время от начала, с")
        ax[2].set_ylabel("Ошибка угла рысканья, рад")
        ax[2].set_title("Ошибка угла рысканья")
        for a in ax:
            a.legend()
            a.grid(True)
        fig.suptitle(self.run_name)
        fig.tight_layout()

        os.makedirs(self.output_dir, exist_ok=True)
        png = os.path.join(self.output_dir, f"{self.run_name}.png")
        fig.savefig(png, dpi=150)
        plt.close(fig)
        np.savez(
            os.path.join(self.output_dir, f"{self.run_name}.npz"),
            gt=gt_np,
            enc=enc_np,
            cmd=cmd_rows_np,
            t=tt,
            pos_err_enc=pos_err_enc,
            pos_err_cmd=pos_err_cmd,
            yaw_err_enc=yaw_err_enc,
            yaw_err_cmd=yaw_err_cmd,
        )
        self.get_logger().info(
            f"Готово: {png}; финальная ошибка положения "
            f"энкодеры {pos_err_enc[-1]:.3f} м, команды {pos_err_cmd[-1]:.3f} м"
        )


def main(args=None):
    rclpy.init(args=args)
    instance = OdometryLogger()
    try:
        while not instance.done and rclpy.ok():
            rclpy.spin_once(instance, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    instance.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
