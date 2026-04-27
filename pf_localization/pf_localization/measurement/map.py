import numpy as np
from math import floor

class Map:
    def __init__(
            self,
            img,
            map_resolution,
            origin,
            OCCUPIED_THRESHHOLD,
            logger
        ):
        self.img = img
        self.map_resolution = map_resolution
        self.origin = np.array(origin)
        self.OCCUPIED_THRESHHOLD = OCCUPIED_THRESHHOLD
        self.logger = logger

    def check_collision_map_frame(self, pos):

        if(np.any(pos < 0) or np.any(pos >= self.img.shape)):
            #hopefully this is never triggered
            self.logger.warn("Forcing collision due to out of bound issue...")
            return True

        #check for a collision in the map frame
        if(self.img[floor(pos[0])][floor(pos[1])] < self.OCCUPIED_THRESHHOLD):

            return True
        
        return False
    
    def check_collision(self, pose):

        #use this to check if the estimate is in collision with anything

        map_pos = (np.array(pose[:2]) - self.origin[:2]) / self.map_resolution

        return self.check_collision_map_frame(map_pos)

    def get_img_index_from_pos(self, wx: np.ndarray, wy: np.ndarray):
        col = ((wx - self.origin[0]) / self.map_resolution).astype(int)
        row = ((wy - self.origin[1]) / self.map_resolution).astype(int)

        rows, cols = self.img.shape
        col = np.clip(col, 0, cols - 1)
        row = np.clip(row, 0, rows - 1)

        return col, row
