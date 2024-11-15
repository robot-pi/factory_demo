from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        # Declare parameters with default values
        self.declare_parameter('initial_pose_x', 0.0)
        self.declare_parameter('initial_pose_y', 0.0)
        self.declare_parameter('initial_pose_z', 0.0)
        self.declare_parameter('initial_pose_roll', 0.0)
        self.declare_parameter('initial_pose_pitch', 0.0)
        self.declare_parameter('initial_pose_yaw', 0.0) 

        self.nav = BasicNavigator()

    def publish_initial_pose(self):
        x = self.get_parameter('initial_pose_x').value
        y = self.get_parameter('initial_pose_y').value
        z = self.get_parameter('initial_pose_z').value
        roll = self.get_parameter('initial_pose_roll').value
        pitch = self.get_parameter('initial_pose_pitch').value
        yaw = self.get_parameter('initial_pose_yaw').value

        initial_pose = PoseStamped()
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = 'map'  # Map frame is typical for Nav2

        # Set the position
        initial_pose.pose.position.x = x
        initial_pose.pose.position.y = y
        initial_pose.pose.position.z = z

        quaternion = quaternion_from_euler(roll, pitch, yaw)
        initial_pose.pose.orientation.x = quaternion[0]
        initial_pose.pose.orientation.y = quaternion[1]
        initial_pose.pose.orientation.z = quaternion[2]
        initial_pose.pose.orientation.w = quaternion[3]

        self.nav.setInitialPose(initial_pose)
        self.nav.waitUntilNav2Active()
        self.get_logger().info('Published initial pose')


def main(args=None):

    rclpy.init(args=args)
    node = InitialPosePublisher()
    node.publish_initial_pose()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
