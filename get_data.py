"""

Get_data.py:  Program contains functions that get data from input files.

Author: Jan Folenta
Email: xfolen00@stud.fit.vutbr.cz
Last update: 19.05.2020

"""

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import numpy as np


"""Function gets coordinates of point from string 'xxx,yyy'"""
def get_coord_from_str(coord):
    coord = coord.split(',')
    x = coord[0]
    y = coord[1]

    return int(x), int(y)


"""Function gets ROI from a text file and returns polygon"""
def get_roi(roi_file):
    polygon_point = list()
    polygon_points = list()
    point_list = list()
    for line in roi_file:
        if line[-1] == "\n":
            line = line[:-1]
        if line == "":
            continue

        points = line.split(',')
        x = int(points[0])
        y = int(points[1])
        p = Point(x, y)
        polygon_point.append(x)
        polygon_point.append(y)
        polygon_points.append(polygon_point)
        point_list.append(p)

    polygon = Polygon([[p.x, p.y] for p in point_list])
    pts = np.array(polygon_points, np.int32)
    pts = pts.reshape((-1, 1, 2))

    return polygon, pts


"""Function gets needed data from text files (like start areas, end areas, directions...)"""
def get_properties(file):
    starts = []
    ends = []
    directions = []
    text_coords = []
    roads = []
    delays = []
    start = False
    end = False
    direction = False

    directions_count = {}
    directions_number = 0

    for line in file:
        if line[-1] == "\n":
            line = line[:-1]

        if line == "start":
            start = True
            end = False
            direction = False
        elif line == "end":
            start = False
            end = True
            direction = False
        elif line == "direction":
            start = False
            end = False
            direction = True

        else:
            if start:
                points = line.split(' ')
                point_list = []
                for point in points:
                    x, y = get_coord_from_str(point)
                    p = Point(x, y)
                    point_list.append(p)
                polygon = Polygon([[p.x, p.y] for p in point_list])
                starts.append(polygon)

            elif end:
                points = line.split(' ')
                point_list = []
                for point in points:
                    x, y = get_coord_from_str(point)
                    p = Point(x, y)
                    point_list.append(p)
                polygon = Polygon([[p.x, p.y] for p in point_list])
                ends.append(polygon)

            elif direction:
                ids = line.split(':')
                ids = ids[1].split(' - ')
                dir_and_coor = ids[0]
                points = ids[1]
                dir_and_coor = dir_and_coor.split(' ')
                directions.append(dir_and_coor[0])
                text_coords.append(dir_and_coor[1])
                points = points.split(' ')
                point_list = []
                for point in points:
                    x, y = get_coord_from_str(point)
                    p = Point(x, y)
                    point_list.append(p)
                polygon = Polygon([[p.x, p.y] for p in point_list])
                roads.append(polygon)
                directions_number += 1

    for idx in range(1, directions_number + 1):
        directions_count[idx] = {}
        directions_count[idx]['car'] = 0
        directions_count[idx]['truck'] = 0

    return starts, ends, directions_count, directions, text_coords, roads


"""Function finds video_id based on name of the video"""
def find_video_id(video):
    video = video + '.mp4'
    file = open('../../datasets/AIC-2020/list_video_id.txt', 'r')
    for line in file:
        line = line.rstrip()
        line = line.split(' ')
        video_id = line[0]
        video_name = line[1]
        if video == video_name:
            return video_id

    return 0

def find_video_length(video):
    video = video + '.mp4'
    file = open('../../datasets/AIC-2020/video_len.txt', 'r')
    for line in file:
        line = line.rstrip()
        line = line.split('\t')
        video_name = line[0]
        fps = line[1]
        length = line[2]
        if video == video_name:
            return length

    return 0
