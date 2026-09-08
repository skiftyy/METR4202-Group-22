import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

from .frontier_helpers import (find_frontiers, group_frontiers, filter_clusters, choose_frontier_goal)
from tf2_ros import Buffer, TransformListener

class Explorer(Node):

    def __init__(self):
        super().__init__('explorer')

        # Subscribe to the SLAM occupancy grid
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.map = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            return (x, y)

        except Exception as e:
            self.get_logger().warn(f'Could not get robot position: {e}')
            return None

    def map_callback(self, msg):
        self.map = msg

        frontiers = find_frontiers(msg)
        raw_clusters = group_frontiers(frontiers)
        clusters = filter_clusters(raw_clusters, 5)

        cluster_sizes = []

        for cluster in clusters:
            cluster_sizes.append(len(cluster))

        self.get_logger().info(
            f'Frontier cells: {len(frontiers)}, '
            f'clusters: {len(clusters)}, '
            f'cluster sizes: {cluster_sizes}'
        )

        robot_position = self.get_robot_position()

        if robot_position is not None:
            self.get_logger().info(
                f'Robot position: '
                f'x={robot_position[0]:.2f}, '
                f'y={robot_position[1]:.2f}'
            )

            frontier, goal = choose_frontier_goal(
                msg,
                clusters,
                robot_position,
                0.4
            )

            if goal is not None:
                self.get_logger().info(
                    f'Chosen frontier cell: {frontier}, '
                    f'goal: x={goal[0]:.2f}, y={goal[1]:.2f}'
                )

            else:
                self.get_logger().warn(
                    'No valid frontier goal found'
                )

def main(args=None):
    rclpy.init(args=args)

    explorer = Explorer()
    rclpy.spin(explorer)

    explorer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()