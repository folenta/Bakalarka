#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""

Process_video.py:  Program will process given video. Program will split given video into frames and in every frame
will detect, track and count vehicles in pre-defined directions. Results will be written into text or video file.

Authors: Jan Folenta, Ing. Jakub Spanhel
Email: xfolen00@stud.fit.vutbr.cz
Last update: 23.05.2020

"""

from __future__ import division, print_function, absolute_import
import os
from timeit import time
import warnings
import cv2
import numpy as np
from PIL import Image
from yolo import YOLO
from shapely.geometry import Point
import argparse
import get_data
import draw


from deep_sort import preprocessing
from deep_sort import nn_matching
from deep_sort.detection import Detection
from deep_sort.tracker import Tracker
from tools import generate_detections as gdet

warnings.filterwarnings('ignore')

from trajectory_processing import find_start_area, find_end_area, find_nearest_end_area, finish_vehicle
from trajectory_processing import find_start, find_direction, get_last_trajectory_point, check_point
from trajectory_processing import make_color, initialize_object, handle_detected, handle_redetected, handle_predicted
from trajectory_processing import find_distance_and_angle_difference, get_bbox_size, delete_last_n_points, find_road
from trajectory_processing import minimalize_roads_possibilities

dataset_path = "../../datasets/AIC-2020"


def get_video_number(video):
    video = video.split('_')
    return int(video[1])


def find_video_basename(video):
    video_name_parts = video.split('_')
    if len(video_name_parts) == 3:
        return video_name_parts[0] + '_' + video_name_parts[1]

    else:
        return video


def main(yolo):

    # Definition of the parameters
    max_cosine_distance = 0.3
    nn_budget = None
    nms_max_overlap = 1.0

    frame_number = 0
    first_frame = True
    empty = True

    objects = {}

    frame_index = -1
    duplicate_id = -1

    parser = argparse.ArgumentParser()
    parser.add_argument("video", help="Name of video (for example cam_1)", type=str)
    parser.add_argument("-o", help="Show output", action="store_true")
    parser.add_argument("-s", help="Save video", action="store_true")
    args = parser.parse_args()

    if args.o:
        show_output = True
    else:
        show_output = False
        
    if args.s:
        save_video = True
    else:
        save_video = False

    video_fullname = args.video
    video_basename = find_video_basename(video_fullname)

    props_file = open(os.path.join(dataset_path, 'cam_props', video_basename + '_prop.txt'), 'r')
    roi_file = open(os.path.join(dataset_path,'ROIs',  video_basename + '.txt'), 'r')
    out_list = open('outputs/text/' + video_fullname + '_out.txt', 'w+')  # file for text output

    video_id = get_data.find_video_id(args.video)

    # deep_sort
    model_filename = 'model_data/mars-small128.pb'
    encoder = gdet.create_box_encoder(model_filename, batch_size=1)

    metric = nn_matching.NearestNeighborDistanceMetric("cosine", max_cosine_distance, nn_budget)
    tracker = Tracker(metric)

    video_capture = cv2.VideoCapture(os.path.join(dataset_path, video_fullname + '.mp4'))

    if save_video:
        # Define the codec and create VideoWriter object
        w = int(video_capture.get(3))
        h = int(video_capture.get(4))
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        out = cv2.VideoWriter('outputs/video/' + video_fullname + '.avi', fourcc, 15, (w, h))

    roi, polygon_points = get_data.get_roi(roi_file)
    starts, ends, directions, dirs, text_coords, roads = get_data.get_properties(props_file)

    fps = 0.0
    while True:
        ret, frame = video_capture.read()  # frame shape 640*480*3
        if not ret:
            break
        t1 = time.time()

        if first_frame:
            first_frame = False

        video_num = get_video_number(args.video)

        image = Image.fromarray(frame[..., ::-1])  # bgr to rgb
        boxs, object_types = yolo.detect_image(image, video_num)
        features = encoder(frame, boxs)

        # score to 1.0 here.
        detections = [Detection(bbox, 1.0, feature) for bbox, feature in zip(boxs, features)]

        # Run non-maxima suppression.
        boxes = np.array([d.tlwh for d in detections])
        scores = np.array([d.confidence for d in detections])
        indices = preprocessing.non_max_suppression(boxes, nms_max_overlap, scores)
        detections = [detections[i] for i in indices]
        object_types = [object_types[i] for i in indices]

        # Call the tracker
        tracker.predict()
        tracker.update(detections, object_types)

        frame_number += 1

        for track in tracker.tracks:
            trajectory_point = list()
            if not track.is_confirmed() or track.time_since_update > 1:
                continue

            track_bbox = track.to_tlbr()

            # Find bounding box centroid
            x = (int(track_bbox[0]) + int(track_bbox[2])) / 2
            y = (int(track_bbox[1]) + int(track_bbox[3])) / 2

            trajectory_point.append(int(x))
            trajectory_point.append(int(y))

            point = Point(int(x), int(y))
            if not roi.contains(point):
                continue

            # Uncomment if you want to display bounding boxes of vehicles
            """if show_output:
                frame = draw.draw_bounding_box(frame, track_bbox, track.object_type, track.track_id)"""
            bbox_size = get_bbox_size(track_bbox)
            road_id = find_road(roads, point)

            # Vehicle is not initialized yet
            if str(track.track_id) not in objects:
                r, g, b = make_color()
                objects[str(track.track_id)] = initialize_object(r, g, b, trajectory_point, track.object_type, road_id)

            # Vehicle is already initialized
            else:
                if check_point(objects[str(track.track_id)]["trajectory"], trajectory_point):
                    if objects[str(track.track_id)]["predicted_length"] > 0:
                        objects[str(track.track_id)]["trajectory"] = delete_last_n_points(objects[str(track.track_id)]["trajectory"], objects[str(track.track_id)]["predicted_length"])
                        objects[str(track.track_id)]["length"] -= objects[str(track.track_id)]["predicted_length"]
                        objects[str(track.track_id)]["predicted_length"] = 0
                    objects[str(track.track_id)]["trajectory"].append(trajectory_point)
                    road_id = minimalize_roads_possibilities(road_id, objects[str(track.track_id)]["road"])
                    objects[str(track.track_id)]["road"] = road_id
                    objects[str(track.track_id)]["bbox_size"] = bbox_size
                    if objects[str(track.track_id)]["state"] == "predicted":
                        objects[str(track.track_id)]["state"] = "detected"
                    if objects[str(track.track_id)]["state"] == "assigned" and objects[str(track.track_id)]["include"] != "-":
                        objects[objects[str(track.track_id)]["include"]]["state"] = "redetected"
                        objects[objects[str(track.track_id)]["include"]]["include"] = str(track.track_id)

                else:
                    if objects[str(track.track_id)]["duplicate_id"] == "-":
                        r, g, b = make_color()
                        objects[str(track.track_id)]["duplicate_id"] = duplicate_id
                        objects[str(objects[str(track.track_id)]["duplicate_id"])] = initialize_object(r, g, b, trajectory_point, track.object_type, road_id)
                        duplicate_id -= 1
                    else:
                        objects[str(objects[str(track.track_id)]["duplicate_id"])]["trajectory"].append(trajectory_point)

                if track.object_type == "truck":
                    objects[str(track.track_id)]["truck_length"] += 1
                    objects[str(track.track_id)]["type"].append('t')
                else:
                    objects[str(track.track_id)]["type"].append('c')

        if show_output:
            # Draw bounding box from detector
            """for det in detections:
                bbox = det.to_tlbr()
                cv2.rectangle(frame, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (255, 0, 0), 2)"""

            draw.draw_counts(frame, directions, text_coords)
            overlay = frame.copy()

        if len(objects) != 0:

            for object_id in objects:
                points = objects[object_id]["trajectory"]

                if not objects[object_id]["finished"]:
                    objects[object_id]["age"] += 1
                    objects[object_id]["TTL"] = 100

                    if objects[object_id]["state"] == "assigned":
                        continue

                    # Vehicle was detected
                    if len(points) > objects[object_id]["length"]:
                        objects[object_id]["length"] += 1

                    # Vehicle was not detected
                    else:
                        # Vehicle is in state 'detected' but was not detected anymore
                        if objects[object_id]["state"] == "detected":
                            objects, ok = handle_detected(objects, object_id)
                            if not ok:
                                continue

                        # Vehicle is in state 'redetected'
                        if objects[object_id]["state"] == "redetected":
                            objects = handle_redetected(objects, object_id)

                        # Vehicle is in state 'predicted'
                        if objects[object_id]["state"] == "predicted":
                            objects = handle_predicted(objects, object_id, ends, roads, roi)

                    if objects[object_id]["length"] > 1:
                        if objects[object_id]["state"] == "redetected":
                            objects = find_distance_and_angle_difference(objects, object_id)

                        else:
                            objects = find_distance_and_angle_difference(objects, object_id)

                    x, y = get_last_trajectory_point(objects[object_id]["trajectory"])
                    point = Point(x, y)

                    # Start area of Vehicle is not found
                    if objects[object_id]["startID"] == 0:
                        objects[object_id]["startID"] = find_start_area(point, starts)

                    # End area of Vehicle is not found
                    if objects[object_id]["endID"] == "-":
                        objects[object_id]["endID"] = find_end_area(point, ends)

                    # Start area and end area are already found
                    if objects[object_id]["startID"] != "-" and objects[object_id]["endID"] != "-":
                        directions, out_list, empty = find_direction(objects[object_id]["trajectory"],
                                                                     objects[object_id]["predicted_length"],
                                                                     objects[object_id]["truck_length"],
                                                                     objects[object_id]["type"],
                                                                     objects[object_id]["startID"],
                                                                     objects[object_id]["endID"],
                                                                     directions, ends, dirs, out_list, frame_number,
                                                                     video_id, empty)
                        objects = finish_vehicle(objects, object_id)

                    # End area is found but start area not
                    if objects[object_id]["startID"] == "-" and objects[object_id]["endID"] != "-":
                        # objects[object_id]["startID"] = find_nearest_start_area(points[0], starts)
                        if objects[object_id]["length"] <= 4:
                            objects = finish_vehicle(objects, object_id)
                            objects[object_id]["TTL"] = 0
                            break
                        objects[object_id]["startID"] = find_start(objects[object_id]["trajectory"], starts, roi)
                        if objects[object_id]["startID"] == "-":
                            objects = finish_vehicle(objects, object_id)
                            objects[object_id]["TTL"] = 0
                            break
                        directions, out_list, empty = find_direction(objects[object_id]["trajectory"],
                                                                     objects[object_id]["predicted_length"],
                                                                     objects[object_id]["truck_length"],
                                                                     objects[object_id]["type"],
                                                                     objects[object_id]["startID"],
                                                                     objects[object_id]["endID"],
                                                                     directions, ends, dirs, out_list, frame_number,
                                                                     video_id, empty)
                        objects = finish_vehicle(objects, object_id)

                    # Last point of trajectory is not in ROI and end area is not found yet
                    if not roi.contains(point) and objects[object_id]["endID"] == "-":
                        objects[object_id]["endID"] = find_nearest_end_area(point, ends)
                        if objects[object_id]["endID"] == "-":
                            objects = finish_vehicle(objects, object_id)
                            objects[object_id]["TTL"] = 0

                        if objects[object_id]["startID"] == "-":
                            # objects[object_id]["startID"] = find_nearest_start_area(points[0], starts)
                            if objects[object_id]["length"] <= 4:
                                objects = finish_vehicle(objects, object_id)
                                objects[object_id]["TTL"] = 0
                                break
                            objects[object_id]["startID"] = find_start(objects[object_id]["trajectory"], starts, roi)
                            if objects[object_id]["startID"] == "-":
                                objects = finish_vehicle(objects, object_id)
                                objects[object_id]["TTL"] = 0
                                break
                        directions, out_list, empty = find_direction(objects[object_id]["trajectory"],
                                                                     objects[object_id]["predicted_length"],
                                                                     objects[object_id]["truck_length"],
                                                                     objects[object_id]["type"],
                                                                     objects[object_id]["startID"],
                                                                     objects[object_id]["endID"],
                                                                     directions, ends, dirs, out_list, frame_number,
                                                                     video_id, empty)
                        objects = finish_vehicle(objects, object_id)

                # Vehicle finished its trajectory - decrement TTL
                else:
                    objects[object_id]["TTL"] -= 1

                # If vehicle is assigned to another vehicle, dont draw its trajectory
                if objects[object_id]["state"] == "assigned":
                    continue

                if show_output:
                    # If TTL is not 0, draw trajectory of vehicle
                    if objects[object_id]["TTL"] > 0:
                        overlay = draw.draw_trajectory(overlay, objects[object_id]["trajectory"], roi, polygon_points, objects[object_id]["color"])

        if show_output:
            cv2.addWeighted(overlay, 0.75, frame, 1 - 0.75, 0, frame)
            cv2.imshow('', frame)

        if save_video:
            out.write(frame)
            frame_index = frame_index + 1

        fps = (fps + (1. / (time.time() - t1))) / 2
        # print fps each 200 frames
        #if frame_number % 200 == 0:
        print("fps= %f" % (fps))
        print("frame number= %d" % frame_number)

        # Press Q to stop!
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    if save_video:
        out.release()

    if show_output:
        cv2.destroyAllWindows()

    video_capture.release()
    out_list.close()


if __name__ == '__main__':

    start_time = time.time()
    main(YOLO())
    end_time = time.time()

    print("Processing time: ", end_time - start_time)
