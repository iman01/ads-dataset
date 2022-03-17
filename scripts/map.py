import cv2
import numpy as np


class CvMap:
    map_res = 0.2  # map resolution m/pixel

    def __init__(self, path, obstacles):
        self.map_path = path

        self.obstacles_corners_map = []  # obstacles in map coordinates
        for single_rectangle in obstacles:
            single_rectangle_map = []
            for each_corner in single_rectangle:
                single_rectangle_map.append([int(each_corner[0] / self.map_res) + 8,
                                             64 - int(each_corner[1] / self.map_res)])
            self.obstacles_corners_map.append(single_rectangle_map)

        self.map_path = [[int(j / self.map_res) for j in i] for i in self.map_path]  # compute pixel from meter
        self.map_path = [[i + 8 for i in self.map_path[0]], [64 - i for i in self.map_path[1]]]  # shift to map

        img_1 = np.zeros([128, 128, 1], dtype=np.uint8)
        img_1.fill(255)

        for i in range(0, len(self.map_path[0])):  # draw path on map
            img_1[self.map_path[1][i], self.map_path[0][i]] = 0

        for single_obstacle_corners_map in self.obstacles_corners_map:
            cv2.fillConvexPoly(img_1, np.array(single_obstacle_corners_map), 1)

        img_1 = cv2.rotate(img_1, cv2.cv2.ROTATE_90_COUNTERCLOCKWISE)
        cv2.imshow('Single Channel Window', img_1)

        cv2.waitKey(0)

        # cv2.destroyAllWindows()






                





