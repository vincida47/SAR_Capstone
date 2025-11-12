#!/usr/bin/env python3
import math, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_srvs.srv import SetBool

from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer, TransformListener

STEP = 2.0
RINGS = 6
DWELL = 1.0

LIFECYCLE_TARGETS = [
    '/controller_server',
    '/planner_server',
    '/bt_navigator',
    '/behavior_server',
    '/waypoint_follower',
    '/velocity_smoother',
]

class Explorer(Node):
    def __init__(self):
        super().__init__('auto_explore_waypoints')

        # TF buffer to check transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav = BasicNavigator()
        self.get_logger().info('Waiting for Nav2 (SLAM mode, no AMCL)…')
        self._wait_for_nav2_active_slam()

        self.paused = False
        self.pause_srv = self.create_service(SetBool, 'auto_explore/pause', self._on_pause)

        # Build and send initial ring of waypoints
        self._send_next_ring()

        # Timer to keep sending rings
        self.create_timer(DWELL, self._tick)

    def _wait_for_nav2_active_slam(self, timeout=60.0):
        """Wait for Nav2 nodes to be ACTIVE and required TFs to be available (SLAM: map->odom, odom->base_link)."""
        deadline = self.get_clock().now().seconds_nanoseconds()[0] + timeout

        # Wait lifecycle ACTIVE
        for name in LIFECYCLE_TARGETS:
            cli = self.create_client(GetState, f'{name}/get_state')
            if not cli.wait_for_service(timeout_sec=10.0):
                self.get_logger().warn(f'{name}/get_state service not available yet')
            req = GetState.Request()
            ok = False
            while self.get_clock().now().seconds_nanoseconds()[0] < deadline:
                future = cli.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                if future.result() and future.result().current_state.label == 'active':
                    ok = True
                    break
                time.sleep(0.5)
            if not ok:
                raise RuntimeError(f'Nav2 node not active: {name}')

        # Wait TFs
        def wait_tf(target, source, sec=10.0):
            t0 = time.time()
            while time.time() - t0 < sec:
                if self.tf_buffer.can_transform(target, source, rclpy.time.Time()):
                    return True
                time.sleep(0.1)
            return False

        if not wait_tf('odom', 'base_link', 10.0):
            raise RuntimeError('Missing TF odom -> base_link (robot_localization/EKF?)')
        if not wait_tf('map', 'odom', 10.0):
            raise RuntimeError('Missing TF map -> odom (Cartographer?)')

        self.get_logger().info('Nav2 active (SLAM) and TFs available.')

    def _on_pause(self, req, res):
        self.paused = bool(req.data)
        res.success = True
        res.message = 'paused' if self.paused else 'running'
        return res

    def _current_pose_in_map(self):
        tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x = tf.transform.translation.x
        p.pose.position.y = tf.transform.translation.y
        p.pose.orientation.w = 1.0
        return p

    def _ring_waypoints(self, center, r):
        wps = []
        for th in [i * (math.pi/4) for i in range(8)]:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = center.pose.position.x + r * math.cos(th)
            ps.pose.position.y = center.pose.position.y + r * math.sin(th)
            ps.pose.orientation.w = 1.0
            wps.append(ps)
        return wps

    def _send_next_ring(self):
        center = self._current_pose_in_map()
        waypoints = []
        for k in range(1, RINGS+1):
            waypoints += self._ring_waypoints(center, STEP*k)
        self.get_logger().info(f'Sending {len(waypoints)} waypoints…')
        self.nav.followWaypoints(waypoints)

    def _tick(self):
        if self.paused:
            return
        status = self.nav.getResult()
        if status == TaskResult.SUCCEEDED:
            self.get_logger().info('Ring complete, generating next ring…')
            self._send_next_ring()
        elif status in (TaskResult.CANCELED, TaskResult.FAILED):
            self.get_logger().warn(f'Path ended with status {status}, retrying new ring…')
            self._send_next_ring()

def main():
    rclpy.init()
    node = Explorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
