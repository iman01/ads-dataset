import matplotlib.pyplot as plt
import numpy as np
import math
import datetime
from path import Path
from visualization import Visualize
from map import Map


class SampleGenerator:

    def __init__(self):
        self.visualize = None
        self.p = Path()                          # create path object
        self.p.generate_curvature_polynomials()  # generate list of polynomials parameters
        self.p.generate_path()

    def is_path_valid(self):
        if self.p.check_path_valid():
            self.p.generate_obstacles_center()

            self.visualize = Visualize(self.p)
            self.visualize.plot_curvature(self.p)
            self.visualize.plot_path(self.p)

            self.p.generate_obstacles_center()
            self.p.get_obstacle_corners()
            self.p.pop_overlapped_obstacles()

            self.visualize.plot_obstacle_center(self.p)
            self.visualize.plot_obstacle_corners(self.p)

            self.visualize.plot_show()
            return True
        else:
            return False


if __name__ == '__main__':

    print('generating data... ')

    num_samples = 20  # number of required samples to generate
    sample_count = 0

    while sample_count < num_samples:  # create dataset
        # get parameters
        sample = SampleGenerator()  # generate map

        if sample.is_path_valid(): sample_count += 1  # count if path is valid

    print('generating data ended.')
