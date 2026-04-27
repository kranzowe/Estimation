import numpy as np
from scipy.ndimage import distance_transform_edt
import cv2
from pf_localization.measurement.map import Map

class LidarLikelihoodField:
    def __init__(self, map: Map, logger, occupied_threshold, sigma=0.2, debug=False):
        self.map = map
        self.logger = logger
        self.occupied_threshold = occupied_threshold
        self.sigma = sigma

        occupied = (self.map.img < self.occupied_threshold).astype(np.uint8)
        dist_px = distance_transform_edt(1 - occupied)
        dist_m = dist_px * self.map.map_resolution
        # Gaussian likelihood
        self.likelihood_field = np.exp(-dist_m**2 / (2.0 * self.sigma**2))
        if debug:
            lf_map = (255*self.likelihood_field/np.max(self.likelihood_field)).astype('uint8')
            cv2.imwrite('tmp/lf.jpg', lf_map)

    def laser_scan_endpoints(self, scan, sample_interval, lidar_resolution, lidar_range):
        idxs = np.arange(0, len(scan.ranges), sample_interval)[:lidar_resolution]
        ray_lengths = np.array(scan.ranges)[idxs]
        angles = scan.angle_min + idxs * scan.angle_increment

        filtered_idxs = np.isfinite(ray_lengths) & (ray_lengths > 0.0) & (ray_lengths < lidar_range)
        ray_lengths = ray_lengths[filtered_idxs]
        angles = angles[filtered_idxs]
        endpoints = np.stack([ray_lengths * np.cos(angles), ray_lengths * np.sin(angles)], axis=1)

        return endpoints
    
    def update_particle_weights(self, particles, scan, sample_interval, lidar_resolution, lidar_range):
        endpoints = self.laser_scan_endpoints(scan, sample_interval, lidar_resolution, lidar_range)
        if len(endpoints) == 0:
            return np.ones(len(particles)) / len(particles)

        xs = particles[:,0]
        ys = particles[:,1]
        thetas = particles[:,2]

        cthetas = np.cos(thetas)[:,None]
        sthetas = np.sin(thetas)[:,None]

        endpoint_xs_body_frame = endpoints[:,0]
        endpoint_ys_body_frame = endpoints[:,1]

        endpoint_xs_world_frame = xs[:,None] + cthetas * endpoint_xs_body_frame - sthetas * endpoint_ys_body_frame
        endpoint_ys_world_frame = ys[:,None] + sthetas * endpoint_xs_body_frame + cthetas * endpoint_ys_body_frame

        px_cols, px_rows = self.map.get_img_index_from_pos(endpoint_xs_world_frame, endpoint_ys_world_frame)

        weights = self.likelihood_field[px_rows, px_cols].sum(axis=1)
        if weights.sum() < 1e-100:
            return np.ones(len(particles)) / len(particles)
        return weights / weights.sum()
