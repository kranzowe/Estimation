#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation


class OdomTFPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.create_subscription(Twist, '/ol_rates', self.ol_rates_cb, 10)

    def ol_rates_cb(self, msg):
        now = self.get_clock().now()
        last_s = self.last_time.seconds_nanoseconds()
        now_s = now.seconds_nanoseconds()
        dt = (now_s[0] + now_s[1] * 1e-9) - (last_s[0] + last_s[1] * 1e-9)
        self.last_time = now

        if dt <= 0.0 or dt > 1.0:
            return

        vx = msg.linear.x
        vyaw = msg.angular.z

        self.yaw += vyaw * dt
        self.x += vx * np.cos(self.yaw) * dt
        self.y += vx * np.sin(self.yaw) * dt

        quat = Rotation.from_euler('z', self.yaw).as_quat()

        # Publish odom topic
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vyaw
        self.odom_pub.publish(odom)

        # Broadcast odom → base_link TF (required by slam_toolbox)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
