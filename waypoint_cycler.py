import rclpy
from rclpy.node import Node

from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseStamped


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

        # Create a publisher for the goal_pose topic
        self.publisher_ = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10
        )

        # Keep track of the number of waypoints sent
        self.waypoint_counter = 0

        # First waypoint
        p0 = PoseStamped()
        p0.header.frame_id = 'map'
        p0.pose.position.x = 1.7
        p0.pose.position.y = -0.5
        p0.pose.orientation.w = 1.0

        # Second waypoint
        p1 = PoseStamped()
        p1.header.frame_id = 'map'
        p1.pose.position.x = -0.6
        p1.pose.position.y = 1.8
        p1.pose.orientation.w = 1.0

        # Store the waypoints
        self.waypoints = [p0, p1]

    def bt_log_callback(self, msg: BehaviorTreeLog):
        # Check the current state of the behaviour tree
        for event in msg.event_log:
            if event.node_name == 'NavigateRecovery' and event.current_status == 'IDLE':
                self.send_waypoint()

    def send_waypoint(self):
        # Keep track of the number of waypoints sent
        self.waypoint_counter += 1

        # Alternate between the two waypoints
        if self.waypoint_counter % 2:
            waypoint = self.waypoints[1]
        else:
            waypoint = self.waypoints[0]

        # Publish the waypoint
        self.publisher_.publish(waypoint)

        # Print information to the terminal
        self.get_logger().info(
            f'Sending waypoint {self.waypoint_counter}: '
            f'x={waypoint.pose.position.x}, '
            f'y={waypoint.pose.position.y}'
        )

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