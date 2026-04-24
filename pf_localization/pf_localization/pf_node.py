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

        self.declare_parameter("lateral_noise_std", 0.05) #meters
        self.declare_parameter("forward_noise_std", 0.2) #meters
        self.declare_parameter("theta_noise_std", np.pi/4) #radians
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
        self.params = ParticleFilterParams()
        self.params.num_particles = self.get_parameter("num_particles").value
        self.filter = ParticleFilter(self.params)
        self.filter.resample(self.check_collision)

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

        self.declare_parameter("debug", True)
        if self.get_parameter("debug").value:
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
        self.filter.resample(self.check_collision)

    def lidar_callback(self, msg):
        self.last_lidar_scan = msg
    
    def measurement_update(self):
        if self.last_lidar_scan is None:
            return
        msg = self.last_lidar_scan
        self.get_logger().warn("Scan received.")
        likelihood_function = lambda err, sigma: norm.pdf(0.0, loc=err, scale=sigma)
        y = [msg.ranges[i] for i in range(0, self.true_lidar_resolution, self.lidar_sample_interval)]
        self.filter.update(y, self.get_measurement, likelihood_function, self.get_logger())
        self.get_logger().warn("Updated.")

        xhat, yhat, thetahat, vhat = self.filter.mmse_estimate()
        self.get_logger().warn("I should be publishing.")
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

                self.map_resolution = map_params["resolution"]
                self.origin = map_params["origin"]
                map_size = np.array(self.img.shape) * self.map_resolution
                self.lower_bound = self.origin[:2]
                self.upper_bound = self.lower_bound + map_size

                self.get_logger().info(f"{self.upper_bound}")
                    
        except Exception as e:
            self.get_logger().error(f"Could not load map {self.map_name}. Please adjust the map_name filename.{e}")
            self.map_loaded = False

    def get_measurement(self, pose):
        #IMPORTANT - this assumes that the lidar rotates counterclockwise!
        #Use this to get a measurement and an uncertainty

        #get a measurement based on the current pose of the lidar
        measurement = np.zeros((self.lidar_resolution, 1), dtype=np.float32)
        
        #determine where the pose is on the map
        map_pos = (np.array(pose[:2]) - self.origin[:2]) / self.map_resolution

        for idx, measurement_angle in enumerate(np.linspace(pose[2], pose[2] + (2*np.pi * (self.lidar_resolution - 1) / self.lidar_resolution) , self.lidar_resolution)):

            collision_found = False
            ray_distance = 0
            
            #unit vector in the direction of the ray
            angle_unit_vector = np.array([cos(measurement_angle), sin(measurement_angle)], dtype=np.float32)

            while(not collision_found):
                #increment the ray distance
                ray_distance = self.increment_ray_distance(map_pos, angle_unit_vector, ray_distance)
            
                #check for a collision in the map
                collision_found = self.check_collision_map_frame(map_pos + ray_distance * angle_unit_vector)

            measurement[idx] = ray_distance

        #scale to bring back to the "real" scale of things
        measurement = measurement * self.map_resolution

        #determine the uncertainty of the measurement
        uncertainty = measurement * self.lidar_relative_uncertainty
        uncertainty = np.maximum(uncertainty, self.lidar_min_uncertainty)

        #catch the case were a measurement greater than the lidar's range is expected
        uncertainty[uncertainty > self.lidar_range * self.lidar_relative_uncertainty] = MEGA_UNCERTAINTY

        return measurement, uncertainty

    def increment_ray_distance(self, map_pos, unit_vector, current_dist):

        #calculate the current ray position
        current_ray_pos = map_pos + unit_vector * current_dist

        #determine the current cell
        current_cell = np.array([floor(current_ray_pos[0]), floor(current_ray_pos[1])], dtype=np.float32)
        
        #determine the next cell in the direction
        next_cell_x = current_cell + np.array([unit_vector[0] / abs(unit_vector[0]), 0], dtype=np.float32)
        next_cell_y = current_cell + np.array([0, unit_vector[1] / abs(unit_vector[1])], dtype=np.float32) 

        #determine if it shorter to the next cell to increment to the x bound or y bound
        dist_x = (next_cell_x[0]  - current_ray_pos[0]) / unit_vector[0]
        dist_y = (next_cell_y[1]  - current_ray_pos[1]) / unit_vector[1]

        if(dist_x < dist_y or abs(unit_vector[1]) < 1e-3) and (abs(unit_vector[0]) > 1e-3):
            return current_dist + dist_x
        else:
            return current_dist + dist_y


    def check_collision_map_frame(self, pos):

        if(np.any(pos < 0) or np.any(pos >= self.img.shape)):
            #hopefully this is never triggered
            self.get_logger().warn("Forcing collision due to out of bound issue...")
            return True

        #check for a collision in the map frame
        if(self.img[floor(pos[0])][floor(pos[1])] < OCCUPIED_THRESHHOLD):

            return True
        
        return False
    
    def check_collision(self, pose):

        #use this to check if the estimate is in collision with anything

        map_pos = (np.array(pose[:2], dtype=np.float32) - self.origin[:2]) / self.map_resolution

        return self.check_collision_map_frame(map_pos)

    
    def get_img_index_from_pos(self, pos):

        #transform to map frame
        map_pos = (np.array(pos, dtype=np.float32) - self.origin[:2]) / self.map_resolution

        if(map_pos[0] < 0):
            map_pos[0] = 0
        elif(map_pos[0] >= self.img.shape[0]):
            map_pos[0] = self.img.shape[0] - 1

        if(map_pos[1] < 0):
            map_pos[1] = 0
        elif(map_pos[1] >= self.img.shape[1]):
            map_pos[1] = self.img.shape[1] - 1

        #get the coordinates
        return np.floor(map_pos)

    
    def display_measurement(self, pose):

        #debug function to display a measurement around a given pose


        #get the measurement and the uncertainty
        measurement, uncertainty = self.get_measurement(pose)

        #convert the map to color
        color_map = self.raw_img.convert('RGB')

        #color scale factor = 255 / (max - min)
        color_scale = 255.0 / (self.lidar_relative_uncertainty * self.lidar_range - self.lidar_min_uncertainty)

        #red is the least certain / green is the most certain
        measurement_colors = np.zeros((3, self.lidar_resolution), dtype=np.uint8)
        for i in range(0, self.lidar_resolution):

            measurement_colors[:, i] = np.array([max(0, min(255, floor(color_scale * uncertainty[i]))), 255 - max(0, min(255, floor(color_scale * uncertainty[i]))), 0], dtype=np.uint8)


        #determine where each measurement happened
        for idx, measurement_angle in enumerate(np.linspace(pose[2], pose[2] + 2*np.pi, self.lidar_resolution)):

            angle_unit_vector = np.array([cos(measurement_angle), sin(measurement_angle)])

            measurement_pos = np.array(pose[:2]) + angle_unit_vector * measurement[idx]

            #get the pixel
            pixel = self.get_img_index_from_pos(measurement_pos)

            color_map.putpixel([int(pixel[1]), int(pixel[0])], (measurement_colors[0, idx], measurement_colors[1, idx], measurement_colors[2, idx]))

        #show the center of the measurement as a blue pixel
        center_pixel = self.get_img_index_from_pos(pose[:2])
        color_map.putpixel([int(center_pixel[1]), int(center_pixel[0])], (0,0,255))

        color_map.show()


    def display_particles(self):
        color_map = self.raw_img.convert('RGB')

        mmse_est = self.filter.mmse_estimate()

        #get the measurement and the uncertainty
        measurement, uncertainty = self.get_measurement(mmse_est)
        #color scale factor = 255 / (max - min)
        color_scale = 255.0 / (self.lidar_relative_uncertainty * self.lidar_range - self.lidar_min_uncertainty)
        #red is the least certain / green is the most certain
        measurement_colors = np.zeros((3, self.lidar_resolution), dtype=np.uint8)
        for i in range(0, self.lidar_resolution):
            measurement_colors[:, i] = np.array([max(0, min(255, floor(color_scale * uncertainty[i]))), 255 - max(0, min(255, floor(color_scale * uncertainty[i]))), 0], dtype=np.uint8)
        #determine where each measurement happened
        for idx, measurement_angle in enumerate(np.linspace(mmse_est[2], mmse_est[2] + 2*np.pi, self.lidar_resolution)):
            angle_unit_vector = np.array([cos(measurement_angle), sin(measurement_angle)])
            measurement_pos = np.array(mmse_est[:2]) + angle_unit_vector * measurement[idx]
            pixel = self.get_img_index_from_pos(measurement_pos)
            color_map.putpixel([int(pixel[1]), int(pixel[0])], (measurement_colors[0, idx], measurement_colors[1, idx], measurement_colors[2, idx]))

        #show the center of the measurement as a blue pixel
        for particle in self.filter.particles:
            particle_pixel = self.get_img_index_from_pos(particle[:2])
            color_map.putpixel([int(particle_pixel[1]), int(particle_pixel[0])], (0,0,255))
        mmse_image = self.get_img_index_from_pos(mmse_est[:2])
        map_est = self.get_img_index_from_pos(self.filter.map_estimate()[:2])
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
