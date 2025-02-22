#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from builtin_interfaces.msg import Time

class GamepadControlNode(Node):
    def __init__(self):
        super().__init__('gamepad_control_node') # output by ros2 node list
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.get_logger().info("Gamepad control node started.")

    def joy_callback(self, msg):
        twist_stamped = TwistStamped()
        # Set the header frame_id
        twist_stamped.header.frame_id = "gamepad"
        # Set the timestamp in the header to current time
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        # Map axes to linear and angular velocity
        twist_stamped.twist.linear.x = msg.axes[1] # left stick up/forward/positive down/backward/negative
        twist_stamped.twist.linear.y = msg.axes[0] # left stick left/right
        if msg.buttons[4]:
            twist_stamped.twist.angular.z = float(msg.buttons[4]) # button 4 counter clockwise/positive value
        if msg.buttons[5]:
            twist_stamped.twist.angular.z = -float(msg.buttons[5]) # button 5 clockwise/negative value
        self.publisher.publish(twist_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = GamepadControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()