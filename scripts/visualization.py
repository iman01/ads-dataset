import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import cmapy
import random


class Visualize:
    def __init__(self, p):
        fig = plt.figure()
        fig.suptitle("Path data")
        self._cd = fig.add_subplot(211)  # curvature-distance graph
        self._cd.set_ylim([-(p.path_max_curvature + 0.02), p.path_max_curvature + 0.02])
        self._cd.set_xlim([0, p.waypoints_distance[-1] + 0.05])
        self._cd.set_xlabel("Distance (m)")
        self._cd.set_ylabel("Curvature")

        # zero horizontal line
        x_line = [0.0, p.waypoints_distance[-1] + 0.05]
        y_line = [0, 0]
        self._cd.plot(x_line, y_line, "red", linewidth=0.2)

        self._po = fig.add_subplot(212)  # path-obstacle graph
        self._po.axis('equal')
        self._po.axis('square')
        self._po.set_xlabel('x (m)')
        self._po.set_ylabel('y (m)')

    def plot_curvature(self, p):
        for i in range(0, p.segment_count):
            t = np.linspace(p.waypoints_distance[i], p.waypoints_distance[i + 1], 100)
            eq_poly = np.poly1d(p.poly_list[i])
            self._cd.plot(t, eq_poly(t), "-")
            self._cd.scatter(p.waypoints_distance, p.waypoints_curvature, c='green')

    def plot_path(self, p):
        self._po.set_xlim([min(p.x_path), max(p.x_path) + 0.5])  # TODO: change this to max and min of obstacle
        self._po.set_ylim([min(p.y_path) - 0.5, max(p.y_path) + 0.5])  # TODO: change this to max and min of obstacle
        self._po.arrow(p.x_path[2], p.y_path[2], p.x_path[3] - p.x_path[2], p.y_path[3] - p.y_path[2], width=0.3,
                       color='blue')
        self._po.arrow(p.x_path[-2], p.y_path[-2], p.x_path[-2] - p.x_path[-3], p.y_path[-2] - p.y_path[-3], width=0.3,
                       color='olive')
        self._po.scatter(p.waypoints_x, p.waypoints_y, s=5.0, c='green')  # draw waypoints on the path
        self._po.plot(p.x_path, p.y_path, "red")  # draw the path

    def plot_obstacle_center(self, p):
        for i in range(len(p.obstacles_center_L)):
            self._po.scatter(p.obstacles_center_L[i][0], p.obstacles_center_L[i][1], s=1.5, c='red')
        for i in range(len(p.obstacles_center_R)):
            self._po.scatter(p.obstacles_center_R[i][0], p.obstacles_center_R[i][1], s=1.5, c='red')

    def plot_obstacle_corners(self, p):
        for single_rectangle in p.obstacles_corners:
            rgb_color = cmapy.color('viridis', random.randrange(0, 256), rgb_order=True)
            rgb_color = [random.randrange(20, 256)/255, random.randrange(20, 256)/255, random.randrange(20, 256)/255]
            for each_corner in single_rectangle:
                self._po.scatter(each_corner[0], each_corner[1], s=1.5, color=rgb_color)

    def plot_show(self):
        # self._po.axis('square')
        self._po.axis('scaled')
        now = datetime.now()
        # plt.savefig("Figure-"+ now.strftime("%Y-%m-%d-%H-%M-%S") + ".png")
        plt.show()
