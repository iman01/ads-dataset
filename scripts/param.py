from dataclasses import dataclass
import numpy as np
import random


@dataclass
class Param:
    # script setting
    visualize = True                 # if the current data should be visualized
    required_samples_count = 100     # what is the number of valid data to be generated
    sample_count = 0                 # just a counter, let it be zero

    # car parameters
    car_wheelbase_m = 2.5
    car_steering_wheel_max_deg = 40.00
    path_max_curvature = 1 / (5.3 - 1.792 / 2)

    # path parameters
    waypoint_count = random.randrange(4, 17, 2)  # minimum, maximum-1,
    segment_count = waypoint_count - 1
    polynomial_count = int(waypoint_count / 2 - 1)

    # list of random curvatures
    waypoints_curvature_random = np.random.uniform(-path_max_curvature, path_max_curvature, int(waypoint_count / 2))
    # as each two consecutive way point has the same curvature only a list of count/2 curvatures is needed

    # creating a list of all curvatures
    waypoints_curvature = []
    for i in range(0, len(waypoints_curvature_random)):
        # duplicate every element after itself
        waypoints_curvature.extend([waypoints_curvature_random[i], waypoints_curvature_random[i]])

    # creating list of random distances between two waypoint
    min_distance = 1.0          # minimum length between each consecutive way points
    max_distance = 3.0          # maximum length between each consecutive way points
    waypoints_distance = [0.0]  # first distance is zero
    for i in range(0, waypoint_count - 1):
        waypoints_distance.append(np.random.uniform(min_distance, max_distance) + waypoints_distance[-1])
    waypoints_distance = [round(num, 2) for num in waypoints_distance]














