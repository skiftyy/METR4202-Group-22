import rclpy
from rclpy.node import Node

from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid


class WaypointCycler(Node):

    def __init__(self):
        super().__init__('waypoint_cycler')

        # Subscribe to the Nav2 behaviour tree log
        self.subscription = self.create_subscription(
            BehaviorTreeLog,
            'behavior_tree_log',
            self.bt_log_callback,
            10
        )

        # Subscribe to the occupancy grid map
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.map = None
        self.nav_idle = False

        # Keep track of frontiers we have already attempted
        self.attempted_frontiers = []

        # Ignore new frontiers within 0.6 m of an attempted frontier
        self.frontier_skip_radius = 0.6

        # Stop the robot about 0.4 m back from the frontier
        self.frontier_standoff = 0.4

        # Store the frontier currently being attempted
        self.current_goal = None

        # Publisher for Nav2 goal poses
        self.publisher_ = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10
        )

    def bt_log_callback(self, msg: BehaviorTreeLog):
        for event in reversed(msg.event_log):

            if event.node_name == 'NavigateRecovery':
                idle = event.current_status == 'IDLE'

                # Navigation has just finished or failed
                if idle and not self.nav_idle:

                    # Mark previous frontier as attempted
                    if self.current_goal is not None:
                        self.attempted_frontiers.append(
                            self.current_goal
                        )
                        self.current_goal = None

                    # Find the next frontier
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
        h = self.map.info.height
        data = self.map.data
        resolution = self.map.info.resolution

        # Convert stand-off distance from metres to map cells
        standoff_cells = max(
            1,
            int(self.frontier_standoff / resolution)
        )

        for i in range(w, len(data) - w):

            # Frontier = free cell directly next to unknown space
            if data[i] == 0 and -1 in [
                data[i - 1],
                data[i + 1],
                data[i - w],
                data[i + w]
            ]:

                frontier_x = i % w
                frontier_y = i // w

                # Convert frontier grid coordinates to world coordinates
                frontier_world_x = (
                    self.map.info.origin.position.x
                    + frontier_x * resolution
                )

                frontier_world_y = (
                    self.map.info.origin.position.y
                    + frontier_y * resolution
                )

                # Ignore frontiers we have already tried
                if self.already_attempted(
                    frontier_world_x,
                    frontier_world_y
                ):
                    continue

                # Determine which direction the unknown space is
                unknown_dx = 0
                unknown_dy = 0

                if data[i - 1] == -1:
                    unknown_dx = -1

                elif data[i + 1] == -1:
                    unknown_dx = 1

                elif data[i - w] == -1:
                    unknown_dy = -1

                elif data[i + w] == -1:
                    unknown_dy = 1

                # Move the goal back away from unknown space
                goal_cell_x = (
                    frontier_x
                    - unknown_dx * standoff_cells
                )

                goal_cell_y = (
                    frontier_y
                    - unknown_dy * standoff_cells
                )

                # Make sure goal stays inside the map
                if (
                    goal_cell_x < 0
                    or goal_cell_x >= w
                    or goal_cell_y < 0
                    or goal_cell_y >= h
                ):
                    continue

                goal_index = (
                    goal_cell_y * w
                    + goal_cell_x
                )

                # Goal must be free space
                if data[goal_index] != 0:
                    continue

                # Convert goal grid coordinates to world coordinates
                goal_x = (
                    self.map.info.origin.position.x
                    + goal_cell_x * resolution
                )

                goal_y = (
                    self.map.info.origin.position.y
                    + goal_cell_y * resolution
                )

                # Create the Nav2 goal
                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.header.stamp = (
                    self.get_clock().now().to_msg()
                )

                goal.pose.position.x = goal_x
                goal.pose.position.y = goal_y
                goal.pose.orientation.w = 1.0

                # Store the frontier location, not the stand-off goal
                self.current_goal = (
                    frontier_world_x,
                    frontier_world_y
                )

                # Publish the waypoint
                self.publisher_.publish(goal)

                self.get_logger().info(
                    f'Frontier at: '
                    f'{frontier_world_x:.2f}, '
                    f'{frontier_world_y:.2f} | '
                    f'Goal: '
                    f'{goal_x:.2f}, '
                    f'{goal_y:.2f}'
                )

                return

        self.get_logger().warn(
            'No untried reachable frontiers found'
        )

    def map_callback(self, msg):
        self.map = msg
        print("MAP")


def main(args=None):
    rclpy.init(args=args)

    waypoint_cmder = WaypointCycler()

    rclpy.spin(waypoint_cmder)

    waypoint_cmder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
