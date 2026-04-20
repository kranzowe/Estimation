import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan

from particle_filter import ParticleFilter, ParticleFilterParams
from dynamics import solve_dyn, process_noise

class ParticleFilterNode(Node):

    def __init__(self):
        super().__init__('pf_localization')

        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(
            Vector3, 'imu/accel', self.imu_callback, 10)
        self.gyro_sub = self.create_subscription(
            Vector3, 'imu/gyro', self.gyro_callback, 10)
        self.estimate_pub = self.create_publisher(
            Odometry, "/state_estimate", 10)
        
        self.predict_dt = 0.2
        self.prediction_schedule = self.create_timer(self.predict_dt, self.predict_callback)

        self.resample_dt = 1.0
        self.resample_schedule = self.create_timer(self.resample_dt, self.resample_callback)

        self.accel_sum = 0.0
        self.turn_rate_sum = 0.0
        self.accel_reads = 0
        self.turn_rate_reads = 0

        self.params = ParticleFilterParams()
        self.filter = ParticleFilter(self.params)
        collision_func = None # TODO: Fill this in
        self.filter.resample(collision_func)
    
    def predict_callback(self):
        vdot = self.accel / self.accel_reads
        thetadot = self.turn_rate / self.turn_rate_reads

        self.accel_sum = 0.0
        self.turn_rate_sum = 0.0
        self.accel_reads = 0
        self.turn_rate_reads = 0

        u = [vdot, thetadot]
        self.filter.predict(solve_dyn, u, process_noise, self.predict_dt)
    
    def resample_callback(self):
        collision_func = None # TODO: Fill this in
        self.filter.resample(collision_func)

    def lidar_callback(self, msg):
        measurement_func = None # TODO: Fill this in
        measurement_err_likelihood = None # TODO: Fill this in
        self.filter.update(msg, measurement_func, measurement_err_likelihood)

        xhat, yhat, thetahat, vhat = self.filter.map_estimate()
        odom = Odometry()
        odom.pose.pose.position.x = xhat
        odom.pose.pose.position.y = yhat
        odom.pose.pose.orientation.z = thetahat
        odom.twist.twist.linear.x = vhat
        self.estimate_pub.publish(odom)

    def imu_callback(self, msg):
        self.accel_sum += msg.x
        self.accel_reads += 1
    
    def gyro_callback(self, msg):
        self.turn_rate_sum += msg.z
        self.turn_rate_reads += 1

def main(args=None):
    rclpy.init(args=args)
    pf = ParticleFilterNode()
    rclpy.spin(pf)
    pf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
