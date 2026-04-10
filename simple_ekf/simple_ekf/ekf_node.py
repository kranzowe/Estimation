import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
import time
import numpy as np

import ekf
import params
import measurement as ms

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        self.ekf = ekf.EKF(
            initial_state=params.x0,
            initial_covariance=params.P0,
            process_noise_cov=params.Q,
            dynamics_func=params.f,
            dynamics_jacobian_func=params.F
        )
        # Command subscriber
        self.command_sub = self.create_subscription(
            Twist,
            '/ol_rates',
            self.cmd_callback,
            10)
        self.control_input = [0, 0]

        self.imu_accel_sub = self.create_subscription(
            Vector3,
            '/imu/accel',
            self.imu_accel_callback,
            10)
        self.imu_gyro_sub = self.create_subscription(
            Vector3,
            '/imu/gyro',
            self.imu_gyro_callback,
            10)
        self.imu_integrator = ms.ImuIntegrator()

        # Estimate publisher
        self.estimate_pub = self.create_publisher(Odometry, "/state_estimate", 10)

        self.timer_period = 0.1
        self.last_update = time.time()
        self.timed_prediction = self.create_timer(self.timer_period, self.prediction_callback)

    def cmd_callback(self, msg):
        # Update control input
        self.control_input = [msg.linear.x, msg.angular.z]

    def prediction_callback(self):
        current_time = time.time()
        dt = current_time - self.last_update
        self.last_update = current_time

        self.ekf.predict(self.control_input, dt)
        estimated_state = self.ekf.get_state()

        y_imu = np.array([self.imu_integrator.get_vel_displacement(self.control_input), self.imu_integrator.get_angle_displacement()])
        self.ekf.update(y_imu, ms.h_imu, ms.H_imu, ms.R_imu)

        Odometry_msg = Odometry()
        Odometry_msg.pose.pose.position.x = estimated_state[0]
        Odometry_msg.pose.pose.position.y = estimated_state[1]
        Odometry_msg.pose.pose.orientation.z = estimated_state[2]
        Odometry_msg.twist.twist.linear.x = estimated_state[3]
        self.estimate_pub.publish(Odometry_msg)

    def imu_accel_callback(self, msg):
        vec = np.array([msg.x, msg.y, msg.z])
        mag = np.linalg.norm(vec)
        self.imu_integrator.update(mag, 0, time.time())

    def imu_gyro_callback(self, msg):
        vec = np.array([msg.x, msg.y, msg.z])
        rot = vec[np.argmax(np.abs(vec))]
        self.imu_integrator.update(0, rot, time.time())

def main(args=None):
    rclpy.init(args=args)

    ekf = EKFNode()

    rclpy.spin(ekf)

    ekf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
