import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints

class PatrollerNode(Node):
    def __init__(self):
        super().__init__('patroller_node')
        
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        
        self.get_logger().info('Waiting for Nav2 FollowWaypoints action server...')
        self._action_client.wait_for_server()
        
        self.last_printed_wp = -1
        self.send_waypoints()

    def send_waypoints(self):
        goal_msg = FollowWaypoints.Goal()
        
        coords = [
            (2.0, 1.0),
            (3.0, -1.0),
            (-2.0, -2.0)
        ]
        
        for x, y in coords:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            goal_msg.poses.append(pose)

        self.get_logger().info('Sending 3 waypoints to Nav2...')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )

    def feedback_callback(self, feedback_msg):
        current_wp = feedback_msg.feedback.current_waypoint
        
        if current_wp != self.last_printed_wp:
            self.get_logger().info(f'Navigating to Waypoint {current_wp + 1}...')
            self.last_printed_wp = current_wp

def main(args=None):
    rclpy.init(args=args)
    node = PatrollerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()