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

    # Calculate distance between GPS coordinates and adjusted points
    distance_to_current = math.sqrt((x_gps - current_point['x'])**2 + (y_gps - current_point['y'])**2)
    distance_to_next = math.sqrt((x_gps - next_point['x'])**2 + (y_gps - next_point['y'])**2)

    # Check if the robot has moved past the current point towards the next point
    return distance_to_next < distance_to_current - radius

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
