import cv2
import matplotlib.pyplot as plt
import numpy as np
import math
import datetime
from param import Param


class SampleGenerator:
    def __init__(self, p):

        print('create one sample')
        self.is_path_valid = True


if __name__ == '__main__':

    p = Param  # get parameters

    print('waypoint_count =', p.waypoint_count)
    print('generating data... ')

    while p.sample_count < p.required_samples_count:
        sample = SampleGenerator(p)
        if sample.is_path_valid: p.required_samples_count += 1

    print('generating data ended.')
