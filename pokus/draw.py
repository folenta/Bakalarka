from shapely.geometry import Point
import cv2
import demo
import get_data
import math


"""Function draws ROI (polygon) to a frame"""
def draw_polygon(frame, polygon_points):
    frame = cv2.polylines(frame, [polygon_points], True, (0, 255, 0), 1)
    return frame


"""Function draws a bounding box and ID of vehicle to a given frame"""
def draw_bounding_box(frame, bbox, obj_type, idx):
    if obj_type == "car":
        cv2.rectangle(frame, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (255, 255, 255),
                      2)  # Nakresli bounding box
        cv2.putText(frame, str(idx), (int(bbox[0]), int(bbox[1])), 0, 5e-3 * 200, (0, 255, 0),
                    2)  # Nakresli id sledovaneho objektu
    if obj_type == "truck":
        cv2.rectangle(frame, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (0, 0, 255),
                      2)  # Nakresli bounding box
        cv2.putText(frame, str(idx), (int(bbox[0]), int(bbox[1])), 0, 5e-3 * 200, (0, 255, 0),
                    2)  # Nakresli id sledovaneho objektu

    return frame


"""Function draws trajectory of moving vehicle with arrow of direction"""
def draw_trajectory(overlay, points, roi, polygon_points, color):
    index = 0
    r, g, b = demo.get_color(color)
    x, y = demo.get_coordinates(points[-1])
    pt = Point(x, y)
    if not roi.contains(pt):
        del points[-1]

    first_access = True
    for point in points:
        x, y = demo.get_coordinates(point)
        pt = Point(x, y)
        if not roi.contains(pt):
            return overlay

        if first_access:
            previous_point = points[0]
            previous_pointX, previous_pointY = demo.get_coordinates(previous_point)
            first_access = False

        overlay = draw_polygon(overlay, polygon_points)

        cv2.line(overlay, (int(float(previous_pointX)), int(float(previous_pointY))),
                 (int(float(x)), int(float(y))),
                 (int(r), int(g), int(b)), 2)

        index += 1

        if index == len(points) and len(points) > 6:
            arrow_point = points[index - 5]
            arrow_pointX, arrow_pointY = demo.get_coordinates(arrow_point)

            angle = math.atan2(int(float(arrow_pointY)) - int(float(y)),
                               int(float(arrow_pointX)) - int(float(x)))
            CV_PI = 3.1415926535897932384626433832795
            degrees_angle = angle * 180 / CV_PI

            sipkaX = round(int(float(x)) + 10 * math.cos((degrees_angle + 30) * CV_PI / 180))
            sipkaY = round(int(float(y)) + 10 * math.sin((degrees_angle + 30) * CV_PI / 180))

            sipkaX2 = round(int(float(x)) + 10 * math.cos((degrees_angle - 30) * CV_PI / 180))
            sipkaY2 = round(int(float(y)) + 10 * math.sin((degrees_angle - 30) * CV_PI / 180))

            cv2.line(overlay, (int(float(sipkaX)), int(float(sipkaY))), (int(float(x)), int(float(y))),
                     (int(r), int(g), int(b)), 2)
            cv2.line(overlay, (int(float(sipkaX2)), int(float(sipkaY2))),
                     (int(float(x)), int(float(y))),
                     (int(r), int(g), int(b)), 2)

        previous_pointX = x
        previous_pointY = y

    return overlay


"""Function draws direction ID, number of cars and number of trucks"""
def draw_counts(frame, directions, text_coords):
    for i in range(0, len(directions)):
        x, y = get_data.get_coord_from_str(text_coords[i])
        cv2.putText(frame, str(i + 1) + "-" + str(directions[i + 1]['car']) + ":" + str(directions[i + 1]['truck']),
                    (x, y), 1, 1.5, (255, 255, 255), 2, cv2.LINE_AA)
