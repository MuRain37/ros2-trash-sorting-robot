"""Motion helpers for RobotTaskServer."""

import time


class MotionMixin:

    def _publish_twist_for(self, goal_handle, msg, duration_sec):
        deadline = time.monotonic() + max(0.0, float(duration_sec))
        while time.monotonic() < deadline:
            if self._is_canceling_without_pause(goal_handle):
                break
            deadline, keep_waiting = self._wait_pause_and_extend_deadline(
                goal_handle,
                deadline,
            )
            if not keep_waiting:
                break
            self._cmd_vel_pub.publish(msg)
            time.sleep(0.05)
        self._publish_stop()

    def _current_yaw(self):
        if self._latest_odom_yaw is not None:
            return self._latest_odom_yaw
        return self._latest_imu_yaw

    def _current_odom_xy(self):
        return self._latest_odom_xy

    def _current_map_xy(self):
        if self._latest_map_pose is None:
            return None
        return self._latest_map_pose.x, self._latest_map_pose.y
