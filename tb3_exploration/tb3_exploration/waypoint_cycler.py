import rclpy
from rclpy.node import Node

from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid


class WaypointCycler(Node):

    def __init__(self):
        super().__init__('waypoint_cycler')

        # Create a subscriber to the behavior_tree_log topic
        self.subscription = self.create_subscription(
            BehaviorTreeLog,
            'behavior_tree_log',
            self.bt_log_callback,
            10
        )

        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.map = None
        self.nav_idle = False

        self.attempted_frontiers = []
        self.frontier_skip_radius = 0.6
        self.current_goal = None

        # Create a publisher for the goal_pose topic
        self.publisher_ = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10
        )

    def bt_log_callback(self, msg: BehaviorTreeLog):
        for event in reversed(msg.event_log):
            if event.node_name == 'NavigateRecovery':
                idle = event.current_status == 'IDLE'

                if idle and not self.nav_idle:

                    if self.current_goal is not None:
                        self.attempted_frontiers.append(
                            self.current_goal
                        )
                        self.current_goal = None

                    self.send_waypoint()

                self.nav_idle = idle
                return

    def already_attempted(self, x, y):
        for old_x, old_y in self.attempted_frontiers:

            distance_squared = (
                (x - old_x) ** 2
                + (y - old_y) ** 2
            )

            if distance_squared < self.frontier_skip_radius ** 2:
                return True

        return False

    def send_waypoint(self):
        if self.map is None:
            return

        w = self.map.info.width
        data = self.map.data

        for i in range(w, len(data) - w):

            # Free cell next to unknown cell
            if data[i] == 0 and -1 in [
                data[i - 1],
                data[i + 1],
                data[i - w],
                data[i + w]
            ]:

                x = i % w
                y = i // w

                goal_x = (
                    self.map.info.origin.position.x
                    + x * self.map.info.resolution
                )

                goal_y = (
                    self.map.info.origin.position.y
                    + y * self.map.info.resolution
                )

                # Ignore frontiers we've already tried
                if self.already_attempted(goal_x, goal_y):
                    continue

                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.header.stamp = (
                    self.get_clock().now().to_msg()
                )

                goal.pose.position.x = goal_x
                goal.pose.position.y = goal_y
                goal.pose.orientation.w = 1.0

                # Remember which goal is currently being attempted
                self.current_goal = (goal_x, goal_y)

                self.publisher_.publish(goal)

                self.get_logger().info(
                    f'Going to frontier: '
                    f'{goal_x:.2f}, {goal_y:.2f}'
                )

                return

        self.get_logger().warn(
            'No untried frontiers found'
        )

    def map_callback(self, msg):
        self.map = msg
        print("MAP")


def main(args=None):
    rclpy.init(args=args)

    # Create a new WaypointCycler node
    waypoint_cmder = WaypointCycler()

    # Execute the node
    rclpy.spin(waypoint_cmder)

    # Shutdown ROS
    waypoint_cmder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
