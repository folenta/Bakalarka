#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""

Trajectory_processing.py:  Program contains functions that process given trajectories (predict next state,
merge trajectories, find start and end area and so on).

Author: Jan Folenta
Email: xfolen00@stud.fit.vutbr.cz
Last update: 03.05.2020

"""


from __future__ import division, print_function, absolute_import

import os
import random
import warnings
import math
from math import sin, cos
from shapely.geometry import Point

warnings.filterwarnings('ignore')


def debug_foo():
    return True


"""Function gets coordinates of point from list [xxx,yyy]"""
def get_coordinates(point):
    x = point[0]
    y = point[1]

    return float(x), float(y)


"""Function finds which start area (polygon) contains given point
   When none of start areas contains points, function return '-'"""
def find_start_area(point, starts):
    idx = 0
    for start in starts:
        idx += 1
        if start.contains(point):
            return idx

    return "-"


"""Function finds which end area (polygon) contains given point
   When none of end areas contains points, function returns '-'"""
def find_end_area(point, ends):
    idx = 0
    for end in ends:
        idx += 1
        if end.contains(point):
            return idx

    return "-"


"""Function finds distance between point and end areas, sorts these areas by the closest distance to the point 
   and returns list of start end IDs"""
def find_nearest_end_area(point, ends):
    distance_vector = list()
    final_distances = list()

    for area in range(0, len(ends)):
        distance = ends[area].exterior.distance(point)
        distance_vector.append(distance)

    sorted_distance = sorted(range(len(distance_vector)), key=lambda k: distance_vector[k])

    for idx in range(len(ends)):
        dist = ends[sorted_distance[idx]].exterior.distance(point)
        # Distance between point a end area have to be less than 100, else ignore
        if dist <= 100:
            final_distances.append(sorted_distance[idx])

    # If end area is not found return '-'
    if not final_distances:
        return "-"

    return final_distances


"""Function finds distance between point and start areas, sorts these areas by the closest distance to the point 
   and returns list of start area IDs"""
def find_nearest_start_area(point, starts):
    if isinstance(point, list):
        x, y = get_coordinates(point)
        point = Point(int(x), int(y))
    distance_vector = []

    for area in range(0, len(starts)):
        distance = starts[area].exterior.distance(point)
        distance_vector.append(distance)

    sorted_distance = sorted(range(len(distance_vector)), key=lambda k: distance_vector[k])
    return sorted_distance


"""Function tries to find start area of vehicle"""
def find_start(points, starts, roi):
    points = points[:5]
    first_point = points[0]
    last_point = points[-1]
    x1, y1 = get_coordinates(first_point)
    x2, y2 = get_coordinates(last_point)
    found = False

    # Calculate distance between first and last point of first 5 points of trajectory
    distance = math.hypot(float(x2) - float(x1), float(y2) - float(y1))
    # If distance is less than 10.0, find nearest start area (because that could indicate that vehicle is not moving at
    # the start of its trajectory)
    if distance < 10.0:
        point = Point(x1, y1)
        return find_nearest_start_area(point, starts)

    length = 0
    sum_distances = 0
    final_distances = list()

    angle = 0
    angles = list()
    # Find angles and distances between first 5 points of trajectory
    for i in range(4):
        x1, y1 = get_coordinates(points[i])
        x2, y2 = get_coordinates(points[i + 1])

        distance = math.hypot(float(x2) - float(x1), float(y2) - float(y1))
        angle = math.atan2(float(y2) - float(y1), float(x2) - float(x1))
        angle = math.degrees(angle)

        angles.append(angle)

        length += 1
        sum_distances += distance

    # Find differences between angles calculated before
    for idx in range(len(angles) - 1):
        old_angle = angles[idx]
        new_angle = angles[idx + 1]
        if old_angle < 0:
            old_angle = 360 - abs(old_angle)
        if new_angle < 0:
            new_angle = 360 - abs(new_angle)

        d = new_angle - old_angle
        if d > 180:
            d -= 360
        if d < -180:
            d += 360

        # If difference is bigger than +-60 degrees, find nearest start area (again...that could indicate that vehicle
        # is not moving at the start of its trajectory or average angle would probably be wrong)
        if d > 60 or d < -60:
            point = Point(x1, y1)
            return find_nearest_start_area(point, starts)

    if not angles:
        angle = math.radians(angle)

    # Find average angle
    else:
        a = b = 0
        for ang in angles:
            ang = math.radians(ang)
            a += cos(ang)
            b += sin(ang)

        angle = math.atan2(b, a)

    angle = math.degrees(angle)
    if angle < 0:
        angle += 180
    else:
        angle -= 180
    angle = math.radians(angle)

    distance = sum_distances / length

    x0, y0 = get_coordinates(first_point)

    # Predict coordinates of point by the average angle and average distance of first 5 points of trajectory
    x = float(x0) + distance * cos(angle)
    y = float(y0) + distance * sin(angle)

    point = Point(x, y)
    point_list = list()
    point_list.append(x)
    point_list.append(y)

    idx = 0
    # Predict points until actual point is out of roi or start area contains point
    while roi.contains(point):
        for start in starts:
            idx += 1
            if start.contains(point):
                found = True
                break

        if found:
            break

        idx = 0
        x0, y0 = get_coordinates(point_list)
        x = float(x0) + distance * cos(angle)
        y = float(y0) + distance * sin(angle)
        point = Point(x, y)
        point_list[:] = []
        point_list.append(x)
        point_list.append(y)

    if not found:
        # Find nearest start area
        distances = find_nearest_start_area(point, starts)
        for idx in range(len(starts)):
            dist = starts[distances[idx]].exterior.distance(point)
            # If distance between start area and point is grater than 100 ignore
            if dist <= 100:
                final_distances.append(distances[idx])

    else:
        final_distances.append(idx - 1)

    # If start area is not found return '-'
    if not final_distances:
        return "-"

    return final_distances


"""Function finds ID of direction based on start area ID and end area ID. Return 0 if that direction does not exist"""
def find_idx(start, end, dirs):
    idx = 0
    direction = str(str(start) + "-" + str(end))

    for d in dirs:
        idx += 1
        if direction == d:
            return idx

    return 0


"""Function finds direction of vehicle and write info (frame_nubmer, direction_id, vehicle_id) to text file"""
def find_direction(points, predicted_length, truck_length, point_types, start, end, directions, ends, dirs, out_list,
                   frame_number, video_id, empty):
    idx = 0
    last_point = points[-1]
    x, y = get_coordinates(last_point)
    point = Point(x, y)

    if isinstance(end, list):
        end = int(end[0]) + 1
        distance = ends[end - 1].exterior.distance(point)
        if distance > 100:
            return directions, out_list

    if isinstance(start, list):
        for i in range(0, len(start)):
            idx = find_idx(start[i] + 1, end, dirs)
            if idx != 0:
                break

    else:
        idx = find_idx(start, end, dirs)

    if idx != 0:
        # Ratio between predicted length length of trajectory and length of trajectory have to be at least 25%
        if predicted_length / len(points) <= 0.75:
            obj_type = find_type(len(points), truck_length, point_types)
            if obj_type == "truck":
                obj_type_id = 2
                count = directions[idx]['truck']
                count += 1
                directions[idx]['truck'] = count

            else:
                obj_type_id = 1
                count = directions[idx]['car']
                count += 1
                directions[idx]['car'] = count

            if empty:
                out_line = video_id + ' ' + str(frame_number) + ' ' + str(idx) + ' ' + str(obj_type_id)
                empty = False

            else:
                out_line = '\n' + video_id + ' ' + str(frame_number) + ' ' + str(idx) + ' ' + str(obj_type_id)

            out_list.write(out_line)

    return directions, out_list, empty


"""Function finds type of vehicle"""
def find_type(length, truck_length, types):
    truck = 0
    car = 0

    if len(types) >= 20:
        types = types[-20:]
        for obj_type in types:
            if obj_type == 't':
                truck += 1
            if obj_type == 'c':
                car += 1

        # If ratio between truck detections and car detections of same vehicle is at least 75% -> truck
        if truck / 20 >= 0.75:
            return "truck"

    truck = 0
    car = 0

    # Sometimes happens that at the beginning of trajectory detector does not know if vehicle is truck or car but at
    # the end (lase 5 points of trajectory) detector is sure that it is truck
    if len(types) >= 5:
        types = types[-5:]
        for obj_type in types:
            if obj_type == 't':
                truck += 1
            if obj_type == 'c':
                car += 1

        if truck >= 5:
            return "truck"
        else:
            return "car"

    return "car"


"""Function find distance between last 2 points of vehicle trajectory"""
def find_distance(points):
    points = points[-2:]

    x1, y1 = get_coordinates(points[0])
    x2, y2 = get_coordinates(points[1])

    distance = math.hypot(float(x2) - float(x1), float(y2) - float(y1))

    if distance <= 0.0:
        distance = float(3.0)

    return distance


"""Function returns a color (r, g, b) from a string"""
def get_color(color):
    color = color.split('-')
    r = color[0]
    g = color[1]
    b = color[2]

    return r, g, b


"""Function find angle between last 2 points of vehicle trajectory"""
def find_angle(points):
    points = points[-2:]

    x1, y1 = get_coordinates(points[0])
    x2, y2 = get_coordinates(points[1])

    angle = math.atan2(float(y2) - float(y1), float(x2) - float(x1))
    angle = math.degrees(angle)

    return angle



"""Function finds average angle between given points"""
def find_average_angle(points, angle):
    angles = list()

    if len(points) >= 5:
        points = points[-5:]
        for i in range(4):
            x1, y1 = get_coordinates(points[i])
            x2, y2 = get_coordinates(points[i + 1])

            angle = math.atan2(float(y2) - float(y1), float(x2) - float(x1))
            angle = math.degrees(angle)

            angles.append(angle)

    else:
        if isinstance(angle, str):
            angle_points = points[-2:]

            x1, y1 = get_coordinates(angle_points[0])
            x2, y2 = get_coordinates(angle_points[1])

            angle = math.atan2(float(y2) - float(y1), float(x2) - float(x1))
            angle = math.degrees(angle)

    if not angles:
        angle = math.radians(angle)
    else:
        a = b = 0
        for ang in angles:
            ang = math.radians(ang)
            a += cos(ang)
            b += sin(ang)

        angle = math.atan2(b, a)

    return angle


"""Function predict next trajectory point"""
def predict_trajectory_point(obj, ends, roads, roi):
    distance_difference = obj["distance_difference"]
    points = obj["trajectory"]
    distance = obj["distance"]
    angle = obj["angle"]
    if len(distance_difference) == 1:
        distance = distance + (float(distance_difference[0]) / 2)

    # This solve the problem that last distance difference can be too big or too small compared to others
    else:
        if not distance_difference:
            distance += 0

        elif distance_difference[-1] < -10:
            distance = distance + (abs(float(distance_difference[-1])))
            del obj["distance_difference"][-1]
            if len(distance_difference) > 1:
                if distance_difference[-1] > 10:
                    distance = distance - (abs(float(distance_difference[-1])))
                    del obj["distance_difference"][-1]
        elif distance_difference[-1] > 10:
            distance = distance - (abs(float(distance_difference[-1])))
            del obj["distance_difference"][-1]
            if len(distance_difference) > 1:
                if distance_difference[-1] < -10:
                    distance = distance + (abs(float(distance_difference[-1])))
                    del obj["distance_difference"][-1]

        elif distance_difference[-1] < -5:
            distance = distance + (abs(float(distance_difference[-1])) / 2)

    obj["distance"] = distance

    last_point = points[-1]
    x0, y0 = get_coordinates(last_point)

    # In case, vehicle is only in one road direction
    if len(obj["road"]) != 1:
        angle = find_average_angle(points, angle)

        if obj["included"] != "-" and obj["included_length"] < 5:
            angle = obj["angle"]
            angle = math.radians(angle)

    else:
        # Find end that belong to road direction
        road_id = int(obj["road"][0]) - 1
        road = roads[road_id]
        intersections = list()

        for end in ends:
            intersection = end.intersection(road)
            percentage = intersection.area / end.area * 100
            intersections.append(percentage)

        idx = intersections.index(max(intersections))
        end = ends[idx]

        end_center_point = list(end.centroid.coords)
        center_x = end_center_point[0][0]
        center_y = end_center_point[0][1]

        dist = math.hypot(float(x0) - float(center_x), float(y0) - float(center_y))

        # In case distance from end area centroid is greater than 200
        if dist > 200:
            # Check angle
            ang = math.atan2(float(center_y) - float(y0), float(center_x) - float(x0))
            ang = math.degrees(ang)
            if ang < 0:
                ang = 360 - abs(ang)
            if angle < 0:
                angle = 360 - abs(angle)

            if not ((angle - 30 < ang < angle + 30) or (ang < angle - 330) or (ang > angle + 330)):
                angle = find_average_angle(points, angle)

            else:
                angle = math.atan2(float(center_y) - float(y0), float(center_x) - float(x0))

        else:
            if distance > 5:
                angle = find_average_angle(points, angle)

                point = Point(x0, y0)
                point_list = list()
                point_list.append(x0)
                point_list.append(y0)
                found = False

                while roi.contains(point):
                    if end.contains(point):
                        found = True
                        break

                    # angle = math.radians(angle)
                    xx, yy = get_coordinates(point_list)
                    x = float(xx) + distance * cos(angle)
                    y = float(yy) + distance * sin(angle)
                    point = Point(x, y)
                    point_list[:] = []
                    point_list.append(x)
                    point_list.append(y)

                if not found:
                    angle = math.atan2(float(center_y) - float(y0), float(center_x) - float(x0))
            else:
                angle = math.atan2(float(center_y) - float(y0), float(center_x) - float(x0))

    # angle = math.radians(angle)
    x = float(x0) + distance * cos(angle)
    y = float(y0) + distance * sin(angle)

    predicted_point = list()
    predicted_point.append(x)
    predicted_point.append(y)

    return obj, predicted_point


"""Function delete last point of trajectory difference"""
def delete_last_difference(distance_difference):
    if not distance_difference:
        return distance_difference

    del distance_difference[-1]

    return distance_difference


"""Function returns last point of trajectory difference"""
def get_last_difference(distance_difference):
    if not distance_difference:
        return 0

    return float(distance_difference[-1])


"""Function returns last point of trajectory"""
def get_last_trajectory_point(points):
    point = points[-1]
    x = point[0]
    y = point[1]

    return float(x), float(y)


"""Function tries to re-identificate vehicle. Tries to find vehicle with new trajectory with similar 
   angle of movement
   Returns ID of found vehicle or '-'"""
def find_object(points, angle, objects, object_id):
    circle_x, circle_y = get_last_trajectory_point(points)
    rad = 150  # Distance of vehicle that is tolerated
    obj_idx = "-"

    for idx in objects:
        # New vehicle trajectory has to have length and age max 3 and cant be already assigned to another vehicle
        if (objects[idx]["length"] == 2 or objects[idx]["length"] == 3) and objects[idx]["age"] <= 3 and not \
                objects[idx]["finished"] and not objects[idx]["state"] == "assigned":
            x1, y1 = get_coordinates(objects[idx]["trajectory"][0])
            x2, y2 = get_coordinates(objects[idx]["trajectory"][-1])
            x = (x1 + x2) / 2
            y = (y1 + y2) / 2
            ob_angle = objects[idx]["angle"]
            if ob_angle < 0:
                ob_angle = 360 - abs(ob_angle)

            if angle < 0:
                angle = 360 - abs(angle)

            if (x - circle_x) * (x - circle_x) + (y - circle_y) * (y - circle_y) <= rad * rad:
                if (angle - 45 < ob_angle < angle + 45) or (ob_angle < angle - 270) or (ob_angle > angle + 270):
                    obj_idx = idx
                    break

    # Tries to prevent truck (with big bbox) being re-identificate as car (with small bbox)
    if obj_idx != "-":
        if objects[object_id]["type"][-1] == 't':
            obj1_size = objects[object_id]["bbox_size"]
            obj2_size = objects[obj_idx]["bbox_size"]

            if obj2_size < (obj1_size / 3):
                obj_idx = "-"

    return obj_idx


"""Function deletes predicted points of trajectory based on number of predicted points"""
def delete_predicted_points(points, length):
    points = points[:len(points) - length]

    return points


"""Function merges trajectories of 2 vehicles"""
def merge_trajectories(objects, idx1, idx2):
    for point in objects[idx2]["trajectory"]:
        objects[idx1]["trajectory"].append(point)
    objects[idx1]["length"] += objects[idx2]["length"]
    objects[idx2]["state"] = "assigned"
    return objects


"""Function checks if distance between first and last point of trajectory is OK (greater than 10)"""
def distance_ok(points):
    x1, y1 = get_coordinates(points[0])
    x2, y2 = get_coordinates(points[-1])

    distance = math.hypot(float(x2) - float(x1), float(y2) - float(y1))

    if distance > 10.0:
        return True

    return False


"""Function calculates difference between 2 angles"""
def calculate_angle_difference(old_angle, new_angle):
    difference = 0
    if (old_angle >= 0 and new_angle >= 0) or (old_angle < 0 and new_angle < 0):
        difference = round(new_angle - old_angle, 2)
    elif old_angle >= 0 and new_angle < 0:
        old_angle = (180 - old_angle) * 2 + old_angle
        new_angle = abs(new_angle)
        difference = round(new_angle - old_angle, 2)
    elif old_angle < 0 and new_angle >= 0:
        old_angle = abs(old_angle)
        new_angle = (180 - new_angle) * 2 + new_angle
        difference = round(new_angle - old_angle, 2)

    return difference


"""Function checks if vehicle is moving or not"""
def check_move(obj):
    points = obj["trajectory"]
    points = points[-5:]

    if len(points) >= 5:
        first_point = points[0]
        last_point = points[-1]
        x1, y1 = get_coordinates(first_point)
        x2, y2 = get_coordinates(last_point)

        distance = math.hypot(float(x2) - float(x1), float(y2) - float(y1))
        if distance < 10.0:
            return False

        angles = list()
        for i in range(4):
            x1, y1 = get_coordinates(points[i])
            x2, y2 = get_coordinates(points[i + 1])

            angle = math.atan2(float(y2) - float(y1), float(x2) - float(x1))
            angle = math.degrees(angle)

            angles.append(angle)

        for idx in range(len(angles) - 1):
            old_angle = angles[idx]
            new_angle = angles[idx + 1]
            if old_angle < 0:
                old_angle = 360 - abs(old_angle)
            if new_angle < 0:
                new_angle = 360 - abs(new_angle)

            d = new_angle - old_angle
            if d > 180:
                d -= 360
            if d < -180:
                d += 360

            if d > 60 or d < -60:
                return False

    return True


"""Function checks if new detected point of vehicle trajectory is not too far from last one 
   (this functions tries to solve one problem of tracker)"""
def check_point(points, point):
    if len(points) == 1:
        return True

    last_x, last_y = get_coordinates(points[-1])
    new_x, new_y = get_coordinates(point)

    distance = math.hypot(float(new_x) - float(last_x), float(new_y) - float(last_y))

    first_x, first_y = get_coordinates(points[-2])
    first_angle = math.atan2(float(last_y) - float(first_y), float(last_x) - float(first_x))
    first_angle = math.degrees(first_angle)

    second_angle = math.atan2(float(new_y) - float(last_y), float(new_x) - float(last_x))
    second_angle = math.degrees(second_angle)

    if first_angle < 0:
        first_angle = 360 - abs(first_angle)
    if second_angle < 0:
        second_angle = 360 - abs(second_angle)

    d = second_angle - first_angle
    if d > 180:
        d -= 360
    if d < -180:
        d += 360

    if distance > 100 and (d > 60 or d < -60):
        return False

    return True


"""Function returns a random color (r, g, b)"""
def make_color():
    r = str(random.randint(0, 255))
    g = str(random.randint(0, 255))
    b = str(random.randint(0, 255))

    return r, g, b


"""Function initialize new vehicle"""
def initialize_object(r, g, b, trajectory_point, obj_type, road_id):
    obj = {}
    obj["color"] = r + '-' + g + '-' + b
    obj["trajectory"] = list()
    obj["trajectory"].append(trajectory_point)
    obj["length"] = 0
    obj["TTL"] = 50
    obj["startID"] = 0
    obj["endID"] = "-"
    obj["finished"] = False
    obj["distance"] = 0
    obj["angle"] = 0
    obj["distance_difference"] = list()
    obj["angle_difference"] = list()
    obj["state"] = "detected"
    obj["predicted_length"] = 0
    obj["include"] = "-"
    obj["included"] = "-"
    obj["included_length"] = 0
    obj["age"] = 0
    obj["type"] = list()
    obj["truck_length"] = 0
    obj["bbox_size"] = 0
    obj["road"] = road_id

    if obj_type == "truck":
        obj["truck_length"] += 1
        obj["type"].append('t')

    else:
        obj["type"].append('c')

    obj["duplicate_id"] = "-"

    return obj


def handle_detected(objects, object_id):
    if objects[object_id]["length"] == 1:
        return objects, False
    dist = find_distance(objects[object_id]["trajectory"])
    length = objects[object_id]["length"]

    if not check_move(objects[object_id]):
        return objects, False
    if length < 3:
        return objects, False
    elif length > 4 and distance_ok(objects[object_id]["trajectory"]):
        objects[object_id]["state"] = "predicted"
    elif dist > 7.5:
        objects[object_id]["state"] = "predicted"
    else:
        return objects, False

    if objects[object_id]["length"] > 3:
        last_difference = get_last_difference(objects[object_id]["distance_difference"])
        objects[object_id]["distance"] = objects[object_id]["distance"] - last_difference
        objects[object_id]["distance_difference"] = delete_last_difference(
            objects[object_id]["distance_difference"])

    return objects, True


def handle_redetected(objects, object_id):
    new_x, new_y = get_last_trajectory_point(
        objects[objects[object_id]["include"]]["trajectory"])
    x, y = get_last_trajectory_point(objects[object_id]["trajectory"])
    if x - 0.1 < new_x < x + 0.1 and y - 0.1 < new_y < y + 0.1:
        last_difference = get_last_difference(objects[object_id]["distance_difference"])
        objects[object_id]["distance"] = objects[object_id]["distance"] - last_difference
        objects[object_id]["distance_difference"] = delete_last_difference(
            objects[object_id]["distance_difference"])
        objects[object_id]["state"] = "predicted"
        objects[object_id]["included"] = objects[object_id]["include"]

    else:
        new_point = list()
        new_point.append(new_x)
        new_point.append(new_y)
        objects[object_id]["trajectory"].append(new_point)
        objects[object_id]["type"].append(objects[objects[object_id]["include"]]["type"][-1])
        road_id = minimalize_roads_possibilities(objects[object_id]["road"],
                                                 objects[objects[object_id]["include"]]["road"])
        objects[object_id]["road_id"] = road_id
        objects[object_id]["length"] += 1
        objects[object_id]["included_length"] += 1

    return objects


def handle_predicted(objects, object_id, ends, roads, roi):
    include = find_object(objects[object_id]["trajectory"], objects[object_id]["angle"], objects, object_id)

    if include == object_id or include == objects[object_id]["include"]:
        objects[object_id]["include"] = "-"
    else:
        objects[object_id]["include"] = include

    if objects[object_id]["include"] != "-":
        objects[object_id]["trajectory"] = delete_predicted_points(
            objects[object_id]["trajectory"], objects[object_id]["predicted_length"])
        objects[object_id]["length"] -= objects[object_id]["predicted_length"]
        objects = merge_trajectories(objects, object_id, objects[object_id]["include"])
        objects[object_id]["predicted_length"] = 0
        objects[object_id]["state"] = "redetected"
        objects[include]["include"] = object_id
    else:
        objects[object_id], predicted_point = predict_trajectory_point(objects[object_id], ends, roads, roi)
        objects[object_id]["trajectory"].append(predicted_point)
        objects[object_id]["predicted_length"] += 1
        objects[object_id]["length"] += 1

    return objects


"""Function finds distance and angle difference"""
def find_distance_and_angle_difference(objects, object_id):
    if objects[object_id]["included"] != "-" and objects[object_id]["included_length"] < 3:
        objects[object_id]["included_length"] = 0
        return objects

    old_distance = objects[object_id]["distance"]
    objects[object_id]["distance"] = find_distance(objects[object_id]["trajectory"])
    new_distance = objects[object_id]["distance"]
    distance_difference = round(new_distance - old_distance, 2)
    objects[object_id]["distance_difference"].append(distance_difference)

    old_angle = objects[object_id]["angle"]
    objects[object_id]["angle"] = find_angle(objects[object_id]["trajectory"])
    new_angle = objects[object_id]["angle"]
    if not isinstance(old_angle, str) and not isinstance(new_angle, str):
        # angle_difference = round(new_angle - old_angle, 2)
        angle_difference = calculate_angle_difference(old_angle, new_angle)
        objects[object_id]["angle_difference"].append(angle_difference)
    else:
        objects[object_id]["angle_difference"].append("-")

    return objects


"""Function gets size of bounding box"""
def get_bbox_size(bbox):
    width = int(bbox[2]) - int(bbox[0])
    height = int(bbox[3]) - int(bbox[1])

    size = width * height

    return size


"""Function deletes lasn N points of given trajectory"""
def delete_last_n_points(points, length):
    for i in range(length):
        del points[-1]

    return points


"""Function finds which road contains given point"""
def find_road(roads, point):
    idx = 0
    road_id = list()
    for road in roads:
        idx += 1
        if road.contains(point):
            road_id.append(idx)

    return road_id


"""Function tries to minimalize roads that contains point"""
def minimalize_roads_possibilities(current_road, previous_road):
    # Aj minula aj sucasna cesta je iba jedna
    if len(current_road) == 1 and len(previous_road) == 1:
        if current_road[0] == previous_road[0]:
            return current_road
        else:
            return []

    # Sucasnych je
    else:
        final_road = set(current_road) & set(previous_road)
        final_road = list(final_road)
        return final_road



def get_video_number(video):
    video = video.split('_')
    return int(video[1])


def find_video_basename(video):
    video_name_parts = video.split('_')
    if len(video_name_parts) == 3:
        return video_name_parts[0] + '_' + video_name_parts[1]

    else:
        return video


def finish_vehicle(objects, object_id):
    objects[object_id]["finished"] = True
    if objects[object_id]["include"] != "-":
        objects[objects[object_id]["include"]]["finished"] = True
        objects[objects[object_id]["include"]]["state"] = "assigned"

    if objects[object_id]["included"] != "-":
        objects[objects[object_id]["included"]]["finished"] = True
        objects[objects[object_id]["included"]]["state"] = "assigned"

    return objects
