from shapely.geometry import Polygon
from shapely.geometry import asPolygon


def rec_to_polygon(rec):
    return Polygon([(rec[0][0], rec[0][1]), (rec[1][0], rec[1][1]), (rec[2][0], rec[2][1]), (rec[3][0], rec[3][1])])


def check_collision(rec1, rec2):
    p1 = rec_to_polygon(rec1)
    p2 = rec_to_polygon(rec2)

    # print('intersect', p1.intersects(p2))
    # print('rec1_x', rec1[0][0])  # [point] , [x or y]
    return p1.intersects(p2)
