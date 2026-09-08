import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose

from .frontier_helpers import (find_frontiers, group_frontiers, filter_clusters, choose_frontier_goal)

from tf2_ros import Buffer, TransformListener


class Explorer(Node):

    def __init__(self):
        super().__init__('explorer')

        # Subscribe to the SLAM occupancy grid
        self.map_subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        self.map = None

        # Get the robot position from TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Nav2 action client used to send navigation goals
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Prevent a new goal being sent while Nav2 is already navigating
        self.navigating = False

        # Store the current navigation goal and goals that Nav2 has failed to reach
        self.current_goal = None
        self.failed_goals = []


    def get_robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            return (x, y)

        except Exception as e:
            self.get_logger().warn(f'Could not get robot position: {e}')
            return None


    def map_callback(self, msg):
        self.map = msg

        # Do not select another frontier while the robot is navigating
        if self.navigating:
            return

        frontiers = find_frontiers(msg)
        raw_clusters = group_frontiers(frontiers)
        clusters = filter_clusters(raw_clusters, 5)

        cluster_sizes = []

        for cluster in clusters:
            cluster_sizes.append(len(cluster))

        self.get_logger().info(
            f'Frontier cells: {len(frontiers)}, clusters: {len(clusters)}, cluster sizes: {cluster_sizes}'
        )

        robot_position = self.get_robot_position()

        if robot_position is not None:
            self.get_logger().info(f'Robot position: x={robot_position[0]:.2f}, y={robot_position[1]:.2f}')

            frontier, goal = choose_frontier_goal(msg, clusters, robot_position, 0.4, self.failed_goals)

            if goal is not None:
                self.get_logger().info(f'Chosen frontier cell: {frontier}, goal: x={goal[0]:.2f}, y={goal[1]:.2f}')

                self.send_goal(goal[0], goal[1])

            else:
                self.get_logger().warn('No valid frontier goal found')


    ## Send the selected frontier goal to Nav2
    def send_goal(self, x, y):

        # Wait until the Nav2 action server is available
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 action server not available')
            return

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # No particular final rotation required yet
        goal_msg.pose.pose.orientation.w = 1.0

        self.navigating = True
        self.current_goal = (x, y)

        self.get_logger().info(f'Sending Nav2 goal: x={x:.2f}, y={y:.2f}')

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)


    ## Check whether Nav2 accepted the goal
    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal')

            if self.current_goal is not None:
                self.failed_goals.append(self.current_goal)
                self.get_logger().warn(f'Blacklisted rejected goal: {self.current_goal}')

            self.current_goal = None
            self.navigating = False
            return

        self.get_logger().info('Nav2 accepted the goal')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)


    ## Called when Nav2 finishes attempting the goal
    def goal_result_callback(self, future):
        result = future.result()

        self.navigating = False

        if result.status == 4:
            self.get_logger().info('Navigation succeeded - selecting next frontier')

        else:
            self.get_logger().warn(f'Navigation failed with status: {result.status}')

            # Remember this location so we do not repeatedly attempt the same area
            if self.current_goal is not None:
                self.failed_goals.append(self.current_goal)
                self.get_logger().warn(f'Blacklisted failed goal: {self.current_goal}')

        self.current_goal = None


def main(args=None):
    rclpy.init(args=args)

    explorer = Explorer()
    rclpy.spin(explorer)

    explorer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()