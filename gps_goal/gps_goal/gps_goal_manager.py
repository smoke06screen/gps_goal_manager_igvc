import csv
import math
import os
import utm
import tf2_ros
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration


class GPSGoalManager(Node):
    def __init__(self):
        super().__init__('gps_goal_manager')
        self.get_logger().info("GPS Goal Manager started")

        # ------------------------------
        # Parameters
        # ------------------------------
        self.goal_tolerance = 1.0  # meters

        # ------------------------------
        # TF
        # ------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ------------------------------
        # Publishers
        # ------------------------------
        self.goal_pub = self.create_publisher(
            PoseStamped, '/global_gps_goal', 10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray, '/gps_waypoints_markers', 10
        )

        # ------------------------------
        # Load + state
        # ------------------------------
        self.raw_waypoints = self.load_waypoints()
        self.map_waypoints = []

        self.current_index = 0
        self.active_goal = None
        self.tf_ready = False

        # ------------------------------
        # Timers
        # ------------------------------
        self.create_timer(0.2, self.check_tf_and_init)
        self.create_timer(0.1, self.monitor_progress)

    # ==================================================
    # Load CSV
    # ==================================================
    def load_waypoints(self):
        pkg_share = get_package_share_directory('gps_goal')
        csv_path = os.path.join(pkg_share, 'waypoints', 'waypoints.csv')

        waypoints = []

        if not os.path.exists(csv_path):
            self.get_logger().error(f"Waypoints CSV not found at: {csv_path}")
            return waypoints

        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) < 2:
                    continue
                try:
                    waypoints.append((float(row[0]), float(row[1])))
                except ValueError:
                    continue

        self.get_logger().info(f"Loaded {len(waypoints)} GPS waypoints")
        return waypoints

    # ==================================================
    # TF readiness + initial processing
    # ==================================================
    def check_tf_and_init(self):
        if self.tf_ready:
            return

        try:
            self.tf_buffer.lookup_transform(
                "map", "utm", Time(), Duration(seconds=0.1)
            )

            self.tf_ready = True
            self.get_logger().info("TF map ↔ utm ready")

            self.process_waypoints()
            self.publish_current_goal()

        except Exception:
            self.get_logger().warn_once("Waiting for map ↔ utm TF...")

    # ==================================================
    # GPS → MAP conversion + RViz markers
    # ==================================================
    def process_waypoints(self):
        from tf2_geometry_msgs import do_transform_point

        # clear before filling (prevents duplication)
        self.map_waypoints.clear()

        markers = MarkerArray()

        for i, (lat, lon) in enumerate(self.raw_waypoints):
            utm_x, utm_y, _, _ = utm.from_latlon(lat, lon)

            utm_pt = PointStamped()
            utm_pt.header.frame_id = "utm"
            utm_pt.header.stamp = Time().to_msg()
            utm_pt.point.x = utm_x
            utm_pt.point.y = utm_y
            utm_pt.point.z = 0.0

            tf = self.tf_buffer.lookup_transform(
                "map", "utm", Time(), Duration(seconds=0.5)
            )

            map_pt = do_transform_point(utm_pt, tf)
            self.map_waypoints.append(map_pt)

            # ------------------------------
            # RViz marker
            # ------------------------------
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()

            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position = map_pt.point

            marker.scale.x = marker.scale.y = marker.scale.z = 0.35

            marker.color.r = 1.0
            marker.color.g = 0.3
            marker.color.b = 0.1
            marker.color.a = 1.0

            markers.markers.append(marker)

        self.marker_pub.publish(markers)
        self.get_logger().info("Waypoints transformed and visualized")

    # ==================================================
    # Publish active goal
    # ==================================================
    def publish_current_goal(self):
        if self.current_index >= len(self.map_waypoints):
            self.get_logger().info("All GPS goals completed")
            return

        goal_pt = self.map_waypoints[self.current_index]

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position = goal_pt.point

        # Intentionally neutral orientation (Nav2 handles heading)
        goal.pose.orientation.w = 1.0

        self.active_goal = goal
        self.goal_pub.publish(goal)

        self.get_logger().info(
            f"Published goal {self.current_index + 1}/{len(self.map_waypoints)}"
        )

    # ==================================================
    # Monitor distance to active goal
    # ==================================================
    def monitor_progress(self):
        if not self.tf_ready or self.active_goal is None:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link", Time(), Duration(seconds=0.1)
            )

            dx = self.active_goal.pose.position.x - tf.transform.translation.x
            dy = self.active_goal.pose.position.y - tf.transform.translation.y

            # Intentionally 2D distance check
            dist = math.hypot(dx, dy)

            if dist < self.goal_tolerance:
                self.get_logger().info(
                    f"Reached goal {self.current_index + 1}"
                )
                self.current_index += 1
                self.publish_current_goal()

        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = GPSGoalManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
