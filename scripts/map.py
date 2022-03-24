import cv2
import numpy as np


class CvMap:
    map_res = 0.2  # map resolution m/pixel

    def __init__(self):
        self.map_path = None
        self.obstacles_corners_map = []  # obstacles in map coordinates
        self.img_1 = np.zeros([128, 128, 1], dtype=np.uint8)
        self.img_1.fill(255)

    def draw_path(self, path):
        self.map_path = path
        self.map_path = [[int(j / self.map_res) for j in i] for i in self.map_path]  # compute pixel from meter
        # self.map_path = [[i + 8 for i in self.map_path[0]], [64 - i for i in self.map_path[1]]]  # shift to map
        self.map_path = [[64 - i for i in self.map_path[1]], [120 - i for i in self.map_path[0]] ]  # shift to map

        for i in range(0, len(self.map_path[0])):  # draw path on map
            self.img_1[self.map_path[1][i], self.map_path[0][i]] = 0

    def draw_obstacles(self, obstacles):
        for single_rectangle in obstacles:
            single_rectangle_map = []
            for each_corner in single_rectangle:
                # single_rectangle_map.append([int(each_corner[0] / self.map_res) + 8,
                #                              64 - int(each_corner[1] / self.map_res)])
                single_rectangle_map.append([64 - int(each_corner[1] / self.map_res),
                                             120 - int(each_corner[0] / self.map_res)])
            self.obstacles_corners_map.append(single_rectangle_map)
        for single_obstacle_corners_map in self.obstacles_corners_map:

            cv2.fillConvexPoly(self.img_1, np.array(single_obstacle_corners_map), 1)

    def draw_path_lane(self, map_lane):
        map_lane_map = []
        for each_point in map_lane:
            map_lane_map.append([64 - int(each_point[1] / self.map_res),
                                         120 - int(each_point[0] / self.map_res)])
        cv2.fillConvexPoly(self.img_1, np.array(map_lane_map), 1)

    def show_map(self):
        cv2.imshow('Single Channel Window', self.img_1)
        cv2.waitKey(0)
        # cv2.destroyAllWindows()
