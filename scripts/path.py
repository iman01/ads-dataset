from dataclasses import dataclass
from datetime import datetime
from collision import check_collision
import numpy as np
import random
import math
import os


def get_4_corners(center, angle, dimensions):
    width = dimensions[0]
    height = dimensions[1]
    return [
        # TOP RIGHT VERTEX:
        [center[0] + ((width / 2) * math.cos(angle)) - ((height / 2) * math.sin(angle)),
         center[1] + ((width / 2) * math.sin(angle)) + ((height / 2) * math.cos(angle))],
        # TOP LEFT VERTEX:
        [center[0] - ((width / 2) * math.cos(angle)) - ((height / 2) * math.sin(angle)),
         center[1] - ((width / 2) * math.sin(angle)) + ((height / 2) * math.cos(angle))],
        # BOTTOM LEFT VERTEX:
        [center[0] - ((width / 2) * math.cos(angle)) + ((height / 2) * math.sin(angle)),
         center[1] - ((width / 2) * math.sin(angle)) - ((height / 2) * math.cos(angle))],
        # BOTTOM RIGHT VERTEX:
        [center[0] + ((width / 2) * math.cos(angle)) + ((height / 2) * math.sin(angle)),
         center[1] + ((width / 2) * math.sin(angle)) - ((height / 2) * math.cos(angle))]

    ]


def obstacle_pose(p1, p2, dis, side):
    obstacle_pos_point = [0.0, 0.0]

    slope = (p2[1] - p1[1]) / (p2[0] - p1[0])
    perp_slope = -1 / slope
    temp = dis * math.sqrt(1 / (1 + perp_slope ** 2))
    if side:
        if slope >= 0.0:
            obstacle_pos_point[0] = p2[0] - temp
            obstacle_pos_point[1] = p2[1] - perp_slope * temp
        else:
            obstacle_pos_point[0] = p2[0] + temp
            obstacle_pos_point[1] = p2[1] + perp_slope * temp
    else:
        if slope >= 0.0:
            obstacle_pos_point[0] = p2[0] + temp
            obstacle_pos_point[1] = p2[1] + perp_slope * temp
        else:
            obstacle_pos_point[0] = p2[0] - temp
            obstacle_pos_point[1] = p2[1] - perp_slope * temp
    return obstacle_pos_point


def get_4_corners_random(human_obstacle_percentage, center_point, dis,
                         obstacles_orientation):  # TODO: make dis to effect on obstacle size
    random_rotation = 0.0
    if np.random.uniform(0, 100.0) <= human_obstacle_percentage:
        dim = np.array([2.0, 2.0])
        random_rotation = obstacles_orientation
    else:
        dim = np.array([np.random.uniform(4.0, 5.5), np.random.uniform(2.0, 3.0)])
        random_rotation = obstacles_orientation + (
                np.random.uniform(-20, 20) / 180) * math.pi  # random rotation of obstacle
    human_corners = get_4_corners(center_point, random_rotation, dim)
    return np.array([human_corners[0], human_corners[1], human_corners[2], human_corners[3]])


@dataclass
class Path:
    # script setting
    visualize_map = True  # if the current data should be visualized
    store_map = True
    sample_count = 0  # just a counter, let it be zero

    # car parameters
    car_wheelbase_m = 2.5
    car_steering_wheel_max_deg = 40.00
    path_max_curvature = 1 / (5.3 - 1.792 / 2)

    min_distance = 1.0  # minimum length between each consecutive way points
    max_distance = 3.0  # maximum length between each consecutive way points

    d_dis = 0.01  # dt*Velocity

    map_safe_length = 22.0 - 2.0  # the allowed length map - shift of map (10*0.2)

    # obstacle parameters
    obstacles_dis_min = 3.0  # how far is each obstacle from next one
    obstacles_dis_max = 8.0

    min_obstacle_dis_to_path = 3.0  # how far from each side of path
    max_obstacle_dis_to_path = 7.0

    human_obstacle_percentage = 60.0  # what percentage of obstacles are human

    # create directory to store the maps
    now = datetime.now()
    path = os.getcwd()
    path += "/ADS-Dataset-" + now.strftime("%Y-%m-%d-%H-%M-%S")

    # os.makedirs(path)

    def __init__(self):

        # path parameters
        self.waypoint_count = random.randrange(4, 17, 2)  # minimum, maximum-1,
        self.segment_count = self.waypoint_count - 1
        self.polynomial_count = int(self.waypoint_count / 2 - 1)

        # list of random curvatures
        waypoints_curvature_random = np.random.uniform(-self.path_max_curvature, self.path_max_curvature,
                                                       int(self.waypoint_count / 2))
        # as each two consecutive way point has the same curvature only a list of count/2 curvatures is needed

        # creating a list of all curvatures
        self.waypoints_curvature = []
        for i in range(0, len(waypoints_curvature_random)):
            # duplicate every element after itself
            self.waypoints_curvature.extend([waypoints_curvature_random[i], waypoints_curvature_random[i]])

        # creating list of random distances between two waypoint

        self.waypoints_distance = [0.0]  # first distance is zero
        for i in range(0, self.waypoint_count - 1):
            self.waypoints_distance.append(
                np.random.uniform(self.min_distance, self.max_distance) + self.waypoints_distance[-1])
        self.waypoints_distance = [round(num, 2) for num in self.waypoints_distance]

        self.poly_list = []

        self.heading = 0.0
        self.x_path = [0.0, 0.0]
        self.y_path = [0.0, 0.0]
        self.waypoints_x = []
        self.waypoints_y = []
        self.map_path = []

        self.obstacles_center_L = []
        self.obstacles_center_R = []

        self.obstacles_orientation_L = []
        self.obstacles_orientation_R = []

        self.obstacles_corners = []  # 4corners or all obstacles
        self.obstacles_dis_from_path_L = []
        self.obstacles_dis_from_path_R = []

    def generate_curvature_polynomials(self):
        for i in range(0, self.polynomial_count):
            x_n = [self.waypoints_distance[i * 2 + 1] - 0.02, self.waypoints_distance[i * 2 + 1] - 0.01,
                   self.waypoints_distance[i * 2 + 1], self.waypoints_distance[i * 2 + 2],
                   self.waypoints_distance[i * 2 + 2] + 0.01, self.waypoints_distance[i * 2 + 2] + 0.02]
            x_n_line = [self.waypoints_distance[i * 2], self.waypoints_distance[i * 2 + 1]]
            # print("x_n_line: ", x_n_line)
            # print("x_n: ", x_n)
            y_n = [self.waypoints_curvature[i * 2 + 1], self.waypoints_curvature[i * 2 + 1],
                   self.waypoints_curvature[i * 2 + 1],
                   self.waypoints_curvature[i * 2 + 2], self.waypoints_curvature[i * 2 + 2],
                   self.waypoints_curvature[i * 2 + 2]]
            y_n_line = [self.waypoints_curvature[i * 2], self.waypoints_curvature[i * 2 + 1]]
            # print("y_n_line: ", y_n_line)
            # print("y_n: ", y_n)
            self.poly_list.append(np.polyfit(x_n_line, y_n_line, 1))
            self.poly_list.append(np.polyfit(x_n, y_n, 3))
        x_n_line_last = [self.waypoints_distance[-2], self.waypoints_distance[-1]]
        y_n_line_last = [self.waypoints_curvature[-2], self.waypoints_curvature[-1]]
        self.poly_list.append(np.polyfit(x_n_line_last, y_n_line_last, 1))
        # print("self.poly_list: ", self.poly_list)

    def generate_path(self):
        for seg in range(0, self.segment_count):
            eq_poly2 = np.poly1d(self.poly_list[seg])
            ds_count = int((self.waypoints_distance[seg + 1] - self.waypoints_distance[seg]) / 0.01)
            self.waypoints_x.append(self.x_path[-1])
            self.waypoints_y.append(self.y_path[-1])
            for i in range(0, ds_count):
                self.heading = self.d_dis * eq_poly2(self.waypoints_distance[seg] + (i * self.d_dis)) + self.heading
                self.x_path.append(self.d_dis * math.cos(self.heading) + self.x_path[-1])
                self.y_path.append(self.d_dis * math.sin(self.heading) + self.y_path[-1])
        self.map_path = [
            self.x_path,
            self.y_path
        ]

    def check_path_valid(self):

        for x_path_elem in self.x_path:
            if x_path_elem > self.map_safe_length:
                return False

        for y_path_elem in self.y_path:
            if y_path_elem < -9.5 or y_path_elem > 9.5:
                return False

        return True

    def generate_obstacles_center(self):
        path_length_m = len(self.x_path) * 0.02

        obstacles_loc_along_path_L = [4.0]
        while obstacles_loc_along_path_L[-1] < path_length_m:
            # place obstacles along the path as long as path exists by random length
            obstacles_loc_along_path_L.append(
                np.random.uniform(self.obstacles_dis_min, self.obstacles_dis_max) + obstacles_loc_along_path_L[-1])
        obstacles_loc_along_path_L[-1] = path_length_m - 0.02  # last obstacle is at the end of the path

        obstacles_loc_along_path_R = [4.0]
        while obstacles_loc_along_path_R[-1] < path_length_m - 0.02:
            obstacles_loc_along_path_R.append(
                np.random.uniform(self.obstacles_dis_min, self.obstacles_dis_max) + obstacles_loc_along_path_R[
                    -1])  # place obstacles along the path as long as path exists by random length
        obstacles_loc_along_path_R[-1] = path_length_m - 0.02  # last obstacle is at the end of the path

        for i in range(len(obstacles_loc_along_path_L)):  # create vector of distances of obstacles from the path
            self.obstacles_dis_from_path_L.append(
                np.random.uniform(self.min_obstacle_dis_to_path, self.max_obstacle_dis_to_path))
        for i in range(len(obstacles_loc_along_path_R)):  # create vector of distances of obstacles from the path
            self.obstacles_dis_from_path_R.append(
                np.random.uniform(self.min_obstacle_dis_to_path, self.max_obstacle_dis_to_path))

        for i in range(len(obstacles_loc_along_path_L)):
            indexP1 = int(obstacles_loc_along_path_L[i] / 0.02)  # index of point on path to find perpendicular line
            indexP2 = abs(int(
                obstacles_loc_along_path_L[i] / 0.02) - 3)  # index of second point on path to find perpendicular line
            self.obstacles_center_L.append(obstacle_pose([self.x_path[indexP1], self.y_path[indexP1]],
                                                         [self.x_path[indexP2], self.y_path[indexP2]],
                                                         self.obstacles_dis_from_path_L[i],
                                                         True))
            self.obstacles_orientation_L.append(math.atan2(self.y_path[indexP1] - self.y_path[indexP2],  # path heading
                                                           self.x_path[indexP1] - self.x_path[indexP2]))
        for i in range(len(obstacles_loc_along_path_R)):
            indexP1 = int(obstacles_loc_along_path_R[i] / 0.02)  # index of point on path to find perpendicular line
            indexP2 = abs(int(obstacles_loc_along_path_R[i] / 0.02) - 3)
            self.obstacles_center_R.append(obstacle_pose([self.x_path[indexP1], self.y_path[indexP1]],
                                                         [self.x_path[indexP2], self.y_path[indexP2]],
                                                         self.obstacles_dis_from_path_R[i],
                                                         False))
            self.obstacles_orientation_R.append(math.atan2(self.y_path[indexP1] - self.y_path[indexP2],  # path heading
                                                           self.x_path[indexP1] - self.x_path[indexP2]))

    def get_obstacle_corners(self):
        for i in range(len(self.obstacles_center_R)):
            self.obstacles_corners.append(
                get_4_corners_random(self.human_obstacle_percentage, self.obstacles_center_R[i],
                                     self.obstacles_dis_from_path_R[i], self.obstacles_orientation_R[i]))
        for i in range(len(self.obstacles_center_L)):
            self.obstacles_corners.append(
                get_4_corners_random(self.human_obstacle_percentage, self.obstacles_center_L[i],
                                     self.obstacles_dis_from_path_L[i], self.obstacles_orientation_L[i]))

    def pop_overlapped_obstacles(self):
        collision_checked = False
        i = 0  # index first rectangle to check
        j = 1  # index first rectangle to check
        while not collision_checked:

            while j < len(self.obstacles_corners):  # index of second rectangle to check collision
                # print('i', i, ' j', j)
                # print('len', len(self.obstacles_corners))

                if check_collision(self.obstacles_corners[i], self.obstacles_corners[j]):
                    self.obstacles_corners.pop(j)
                    j = j - 1
                    # print('pop')
                j += 1

            i += 1
            j = i + 1
            if i >= len(self.obstacles_corners) +1:
                print('collision_checked')
                collision_checked = True
