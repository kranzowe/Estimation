#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import os
import yaml
import numpy as np
from math import floor, cos, sin
# from pathlib import Path
from scipy.stats import norm

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
from PIL import Image

from particle_filter import ParticleFilter, ParticleFilterParams
from ament_index_python import get_package_share_directory
from dynamics import solve_dyn, process_noise, NoiseParams

from pf_localization.measurement.map import Map
from pf_localization.measurement.llf import LidarLikelihoodField
from pf_localization.measurement.ray_tracing import RayTracer

import json
import os

from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge

OCCUPIED_THRESHHOLD = 0.9 * 255
MEGA_UNCERTAINTY = 1000

class ParticleFilterNode(Node):

    def __init__(self):
        super().__init__('pf_localization')

        self.declare_parameter("true_lidar_resolution", 720)
        self.declare_parameter("lidar_resolution", 60) #measurement per rotation
        self.true_lidar_resolution = self.get_parameter("true_lidar_resolution").value
        self.lidar_resolution = self.get_parameter("lidar_resolution").value
        self.lidar_sample_interval = int(self.true_lidar_resolution / self.lidar_resolution) 
        
        self.declare_parameter("map_filename", "Best_map")
        self.map_name = self.get_parameter("map_filename").value

        #must be less than the range of the lidar
        self.declare_parameter("lidar_range", 10.0)
        self.lidar_range = self.get_parameter("lidar_range").value

        #fraction of the measured distance
        self.declare_parameter("lidar_relative_uncertainty", 0.0025)
        self.lidar_relative_uncertainty = self.get_parameter("lidar_relative_uncertainty").value

        self.declare_parameter("lidar_min_uncertainty", 0.005) #meters
        self.lidar_min_uncertainty = self.get_parameter("lidar_min_uncertainty").value

        self.declare_parameter("lateral_noise_std", 0.02) #meters
        self.declare_parameter("forward_noise_std", 0.1) #meters
        self.declare_parameter("theta_noise_std", np.pi/8) #radians
        self.declare_parameter("v_noise_std", 0.05) #m/s
        self.noise_params = NoiseParams(
            self.get_parameter("lateral_noise_std").value,
            self.get_parameter("forward_noise_std").value,
            self.get_parameter("theta_noise_std").value,
            self.get_parameter("v_noise_std").value
        )

        self.map_loaded = False
        self.load_map()

        self.declare_parameter("num_particles", 100)
        self.declare_parameter("x0_pos", [-28.0, 7.0])
        self.declare_parameter("x0_spread", 1.0)
        x0_pos = self.get_parameter("x0_pos").value
        x0_spread = self.get_parameter("x0_spread").value

        self.params = ParticleFilterParams()
        self.params.num_particles = self.get_parameter("num_particles").value
        self.params.x0_min = [x0_pos[0]-x0_spread, x0_pos[1]-x0_spread, -np.pi, 0]
        self.params.x0_max = [x0_pos[0]+x0_spread, x0_pos[1]+x0_spread, np.pi, 0]
        self.filter = ParticleFilter(self.params)
        self.filter.resample(self.map.check_collision)

        self.accel_sum = 0.0
        self.turn_rate_sum = 0.0
        self.accel_reads = 0
        self.turn_rate_reads = 0
        self.last_lidar_scan = None

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

        self.measurement_dt = 0.5
        self.prediction_schedule = self.create_timer(self.measurement_dt, self.measurement_update)

        self.resample_dt = 1.0
        self.resample_schedule = self.create_timer(self.resample_dt, self.resample_callback)
        
        self.create_timer(1.0, self.param_cb)

        # Debug mode is used for testing off of the Pi
        self.declare_parameter("debug", False)
        if self.get_parameter("debug").value:
            scan_path = os.path.join(get_package_share_directory('pf_localization'), 'data/laserscan.json')
            with open(scan_path, 'r') as f:
                lidar_scan_load = json.load(f)
                msg = LaserScan()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.angle_min = float(lidar_scan_load['angle_min'])
                msg.angle_max = float(lidar_scan_load['angle_max'])
                msg.angle_increment = float(lidar_scan_load['angle_increment'])
                msg.ranges = [float(r) if r is not None else float('inf') for r in lidar_scan_load['ranges']]
                self.last_lidar_scan = msg

        self.declare_parameter("visualize", True)
        if self.get_parameter("visualize").value:
            self.vis_pub = self.create_publisher(ROSImage, '/particle_filter/visualization', 10)
            self.bridge = CvBridge()
            self.visualization_dt = 0.5
            self.vis_schedule = self.create_timer(self.visualization_dt, self.display_particles)
    
    def predict_callback(self):
        vdot = self.accel_sum / self.accel_reads if self.accel_reads > 0 else 0
        thetadot = self.turn_rate_sum / self.turn_rate_reads if self.turn_rate_reads > 0 else 0

        self.accel_sum = 0.0
        self.turn_rate_sum = 0.0
        self.accel_reads = 0
        self.turn_rate_reads = 0

        u = [vdot, thetadot]

        noise_func = lambda p, dt: process_noise(p, dt, self.noise_params)
        self.filter.predict(solve_dyn, u, noise_func, self.predict_dt)
    
    def resample_callback(self):
        self.filter.resample(self.map.check_collision)

    def lidar_callback(self, msg):
        self.last_lidar_scan = msg
    
    def measurement_update(self):
        if self.last_lidar_scan is None:
            return
        msg = self.last_lidar_scan

        # Ray Tracing Measurement Update
        # likelihood_function = lambda err, sigma: norm.pdf(0.0, loc=err, scale=sigma)
        # y = [msg.ranges[i] for i in range(0, self.true_lidar_resolution, self.lidar_sample_interval)]
        # self.filter.update(y, self.get_measurement, likelihood_function, self.get_logger())

        weights = self.llf.update_particle_weights(
            np.array(self.filter.particles),
            msg,
            self.lidar_sample_interval,
            self.lidar_resolution,
            self.lidar_range
        )
        self.filter.update_weights(weights)

        xhat, yhat, thetahat, vhat = self.filter.mmse_estimate()
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
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

    def load_map(self):

        #get the filepath for the map
        file_path_map = os.path.join(get_package_share_directory("pf_localization"), "maps", self.map_name + ".pgm")
        file_path_params = os.path.join(get_package_share_directory("pf_localization"), "maps", self.map_name + ".yaml")

        #load in the map
        self.raw_img = Image.open(file_path_map)
        self.img = np.asarray(self.raw_img)
        self.img = np.reshape(self.img, (self.raw_img.size[1], self.raw_img.size[0]))
        self.map_loaded = True

        #load in the yaml
        try:
            with open(file_path_params, "r") as param_file:
                map_params = yaml.safe_load(param_file)
                self.map = Map(
                    img=self.img,
                    map_resolution=map_params["resolution"],
                    origin=map_params["origin"],
                    OCCUPIED_THRESHHOLD=OCCUPIED_THRESHHOLD,
                    logger=self.get_logger()
                )
                map_size = np.array(self.img.shape) * self.map.map_resolution
                lower_bound = self.map.origin[:2]
                upper_bound = lower_bound + map_size
                self.get_logger().info(f"Map Initialized with bounds: {lower_bound}, {upper_bound}")
            self.llf = LidarLikelihoodField(
                map=self.map,
                sigma=self.lidar_min_uncertainty * 25,   # tune this
                occupied_threshold=int(255*0.4), # tune this
                logger=self.get_logger()
            )
                    
        except Exception as e:
            self.get_logger().error(f"Could not load map {self.map_name}. Please adjust the map_name filename.{e}")
            self.map_loaded = False


    def display_particles(self):
        color_map = self.raw_img.convert('RGB')

        mmse_est = self.filter.mmse_estimate()

        # #get the measurement and the uncertainty

        if self.get_parameter("debug").value:
            ray_tracer = RayTracer(
                map=self.map,
                lidar_resolution=self.lidar_resolution,
                lidar_relative_uncertainty=self.lidar_relative_uncertainty,
                lidar_min_uncertainty=self.lidar_min_uncertainty,
                lidar_range=self.lidar_range,
                MEGA_UNCERTAINTY=MEGA_UNCERTAINTY
            )
            ray_tracer.display_measurement(mmse_est, color_map)

        #show the center of the measurement as a blue pixel
        for particle in self.filter.particles:
            particle_pixel = self.map.get_img_index_from_pos(particle[0], particle[1])
            color_map.putpixel([int(particle_pixel[1]), int(particle_pixel[0])], (0,0,255))
        mmse_image = self.map.get_img_index_from_pos(mmse_est[0], mmse_est[1])
        map_est_pos = self.filter.map_estimate()
        map_est = self.map.get_img_index_from_pos(map_est_pos[0], map_est_pos[1])
        color_map.putpixel([int(map_est[1]), int(map_est[0])], (255,125,0))
        color_map.putpixel([int(mmse_image[1]), int(mmse_image[0])], (255,0,0))
        cv_image = np.array(color_map)
        msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="rgb8")
        self.vis_pub.publish(msg)


    def param_cb(self):
        self.lidar_resolution = self.get_parameter("lidar_resolution").value #measurement per rotation
        self.lidar_range = self.get_parameter("lidar_range").value
        self.lidar_relative_uncertainty = self.get_parameter("lidar_relative_uncertainty").value
        self.lidar_min_uncertainty = self.get_parameter("lidar_min_uncertainty").value
        self.noise_params = NoiseParams(
            self.get_parameter("lateral_noise_std").value,
            self.get_parameter("forward_noise_std").value,
            self.get_parameter("theta_noise_std").value,
            self.get_parameter("v_noise_std").value
        )
        

def main(args=None):
    rclpy.init(args=args)
    pf = ParticleFilterNode()
    rclpy.spin(pf)
    pf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
