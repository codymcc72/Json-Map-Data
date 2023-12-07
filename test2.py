#!/usr/bin/env python3

import os
import rospy
from sensor_msgs.msg import NavSatFix
import json
import math

# Class to represent map data from JSON
class JsonDataMap:
    def __init__(self, json_data):
        # Initialize attributes to store map segments
        self.datum = json_data['datum']
        self.points = json_data['points']

    # Adjust coordinates using the datum information
    def adjust_coordinates(self, point):
        adjusted_x = point['head']['position']['x'] + self.datum['longitude']
        adjusted_y = point['head']['position']['y'] + self.datum['latitude']
        return {'x': adjusted_x, 'y': adjusted_y}

def has_passed_point(gps_data, current_point, next_point, radius):
    x_gps = gps_data.longitude
    y_gps = gps_data.latitude

    # Calculate the vector from the current point to the GPS coordinates
    vector_to_gps = {'x': x_gps - current_point['x'], 'y': y_gps - current_point['y']}

    # Calculate the vector along the path from the current point to the next point
    path_vector = {'x': next_point['x'] - current_point['x'], 'y': next_point['y'] - current_point['y']}

    # Calculate the dot product of the two vectors
    dot_product = vector_to_gps['x'] * path_vector['x'] + vector_to_gps['y'] * path_vector['y']

    # Calculate the magnitude of the path vector
    path_magnitude = math.sqrt(path_vector['x']**2 + path_vector['y']**2)

    # Calculate the projection of the vector to GPS coordinates onto the path vector
    projection = dot_product / path_magnitude

    # Check if the projection is within the bounds of the path
    return 0 <= projection <= path_magnitude and math.sqrt(vector_to_gps['x']**2 + vector_to_gps['y']**2) < radius

def gps_callback(gps_data, json_data_map, radius, last_passed_point_index):
    # GPS coordinates
    x_gps = gps_data.longitude
    y_gps = gps_data.latitude

    # Iterate through map points to find if GPS coordinates have passed any point
    for i in range(len(json_data_map.points) - 1):
        current_point = json_data_map.adjust_coordinates(json_data_map.points[i])
        next_point = json_data_map.adjust_coordinates(json_data_map.points[i + 1])

        # Check if the robot has moved past the current point towards the next point
        if has_passed_point(gps_data, current_point, next_point, radius):
            # Check if it's a different point from the last one passed
            if i != last_passed_point_index:
                last_passed_point_index = i

                print("Point {} Passed".format(i))
                # You may want to add additional logic or processing here

            break  # Exit the loop once a passing point is found

    return last_passed_point_index

def gps_listener(json_data_map):
    rospy.init_node('gps_listener', anonymous=True)

    # Variable to track the index of the last passed point
    last_passed_point_index = -1

    rospy.Subscriber('tric_navigation/gps/head_data', NavSatFix, lambda gps_data: gps_callback(gps_data, json_data_map, radius, last_passed_point_index))
    rospy.spin()

if __name__ == '__main__':
    # Get the path to the JSON map file (assuming it's in the 'json_maps_folder' within the 'src' directory)
    script_dir = os.path.dirname(os.path.realpath(__file__))
    map_file_path = os.path.join(script_dir, 'json_map_plot', 'testrow.json')

    # Load JSON data
    with open(map_file_path, 'r') as file:
        json_data = json.load(file)

    # Initialize JsonDataMap with the loaded JSON data
    json_data_map = JsonDataMap(json_data)

    # Initialize GPS listener with the JsonDataMap instance
    gps_listener(json_data_map)
