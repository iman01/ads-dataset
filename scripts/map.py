import cv2


class Map:
    map_res = 0.2  # map resolution m/pixel


    def __init__(self, map_path):
        self.map_path = map_path




    def check_path_valid(self):

        for x_path_elem in self.map_path[0]:
            if x_path_elem > self.map_safe_length:
                return False

        for y_path_elem in self.map_path[1]:
            if y_path_elem < -9.5 or y_path_elem > 9.5:
                return False

        return True


    # def generate_map(self, p):


                





