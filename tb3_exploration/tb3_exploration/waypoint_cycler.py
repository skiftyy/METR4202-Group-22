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

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.map = None
        self.nav_idle = False

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
                    self.send_waypoint()

                self.nav_idle = idle
                return

    def send_waypoint(self):
        if self.map is None:
            return

        w = self.map.info.width
        data = self.map.data

        for i in range(w, len(data)-w):
            if data[i] == 0 and -1 in [data[i-1], data[i+1], data[i-w], data[i+w]]:
                x, y = i % w, i // w

                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.pose.position.x = self.map.info.origin.position.x + x*self.map.info.resolution
                goal.pose.position.y = self.map.info.origin.position.y + y*self.map.info.resolution
                goal.pose.orientation.w = 1.0

                self.publisher_.publish(goal)
                self.get_logger().info(f'Frontier: {goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}')
                return

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