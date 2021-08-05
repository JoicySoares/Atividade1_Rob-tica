import rclpy
import numpy as np
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Pose,
            'setpoint_pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, 'setpoint_euler', 10)

    def listener_callback(self, msg):
        quaternion = msg.orientation
        linear = msg.position
        roll, pitch, yaw = self.euler_from_quaternion(quaternion)
        
        self.get_logger().info('Linear: ("%s",' % str(linear.x))
        self.get_logger().info('         "%s",' % str(linear.y))
        self.get_logger().info('         "%s")' % str(linear.z))

        self.get_logger().info('Angular: ("%s",' % str(roll))
        self.get_logger().info('          "%s",' % str(pitch))
        self.get_logger().info('          "%s")' % str(yaw))
        
        "Twist msg"
        tmsg = Twist()
        tmsg.linear.x = linear.x
        tmsg.linear.y = linear.y
        tmsg.linear.z = linear.z
        tmsg.angular.x = roll
        tmsg.angular.y = pitch
        tmsg.angular.z = yaw
        
        self.publisher_.publish(tmsg)
        
    def euler_from_quaternion(self, quaternion):
        """Converts quaternion to euler roll, pitch, yaw.
        quaternion = [w, x, y, z] """

        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()