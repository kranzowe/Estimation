import numpy as np
from math import sin, cos, floor
from pf_localization.measurement.map import Map

class RayTracer:
    def __init__(
            self,
            map: Map,
            lidar_resolution,
            lidar_relative_uncertainty,
            lidar_min_uncertainty,
            lidar_range,
            MEGA_UNCERTAINTY
    ):
        self.map = map
        self.lidar_resolution = lidar_resolution
        self.map_resolution = map.map_resolution
        self.origin = map.origin
        self.lidar_relative_uncertainty = lidar_relative_uncertainty
        self.lidar_min_uncertainty = lidar_min_uncertainty
        self.lidar_range = lidar_range
        self.MEGA_UNCERTAINTY = MEGA_UNCERTAINTY


    def get_measurement(self, pose):
        #IMPORTANT - this assumes that the lidar rotates counterclockwise!
        #Use this to get a measurement and an uncertainty

        #get a measurement based on the current pose of the lidar
        measurement = np.zeros((self.lidar_resolution, 1))
        
        #determine where the pose is on the map
        map_pos = (np.array(pose[:2]) - self.origin[:2]) / self.map_resolution

        for idx, measurement_angle in enumerate(np.linspace(pose[2], pose[2] + (2*np.pi * (self.lidar_resolution - 1) / self.lidar_resolution) , self.lidar_resolution)):

            collision_found = False
            ray_distance = 0
            
            #unit vector in the direction of the ray
            angle_unit_vector = np.array([cos(measurement_angle), sin(measurement_angle)])

            while(not collision_found):
                #increment the ray distance
                ray_distance = self.increment_ray_distance(map_pos, angle_unit_vector, ray_distance)
            
                #check for a collision in the map
                collision_found = self.map.check_collision_map_frame(map_pos + ray_distance * angle_unit_vector)

            measurement[idx] = ray_distance

        #scale to bring back to the "real" scale of things
        measurement = measurement * self.map_resolution

        #determine the uncertainty of the measurement
        uncertainty = measurement * self.lidar_relative_uncertainty
        uncertainty = np.maximum(uncertainty, self.lidar_min_uncertainty)

        #catch the case were a measurement greater than the lidar's range is expected
        uncertainty[uncertainty > self.lidar_range * self.lidar_relative_uncertainty] = self.MEGA_UNCERTAINTY

        return measurement, uncertainty

    def increment_ray_distance(self, map_pos, unit_vector, current_dist):

        #calculate the current ray position
        current_ray_pos = map_pos + unit_vector * current_dist

        #determine the current cell
        current_cell = np.array([floor(current_ray_pos[0]), floor(current_ray_pos[1])])
        
        #determine the next cell in the direction
        next_cell_x = current_cell + np.array([unit_vector[0] / abs(unit_vector[0]), 0])
        next_cell_y = current_cell + np.array([0, unit_vector[1] / abs(unit_vector[1])]) 

        #determine if it shorter to the next cell to increment to the x bound or y bound
        dist_x = (next_cell_x[0]  - current_ray_pos[0]) / unit_vector[0]
        dist_y = (next_cell_y[1]  - current_ray_pos[1]) / unit_vector[1]

        if(dist_x < dist_y or abs(unit_vector[1]) < 1e-3) and (abs(unit_vector[0]) > 1e-3):
            return current_dist + dist_x
        else:
            return current_dist + dist_y
    
    def display_measurement(self, pose, color_map):
        #get the measurement and the uncertainty
        measurement, uncertainty = self.get_measurement(pose)

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
            pixel = self.map.get_img_index_from_pos(measurement_pos[0], measurement_pos[1])

            color_map.putpixel([int(pixel[1]), int(pixel[0])], (measurement_colors[0, idx], measurement_colors[1, idx], measurement_colors[2, idx]))

        #show the center of the measurement as a blue pixel
        center_pixel = self.map.get_img_index_from_pos(pose[0], pose[1])
        color_map.putpixel([int(center_pixel[1]), int(center_pixel[0])], (0,0,255))

        return color_map