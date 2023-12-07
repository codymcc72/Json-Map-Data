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

def gps_callback(gps_data, json_data_map, radius, passed_point):
    # GPS coordinates
    x_gps = gps_data.longitude
    y_gps = gps_data.latitude

    # Iterate through map points to find if GPS coordinates are close to any point
    for point in json_data_map.points:
        adjusted_point = json_data_map.adjust_coordinates(point)
        
        # Calculate distance between GPS coordinates and adjusted point
        distance = math.sqrt((x_gps - adjusted_point['x'])**2 + (y_gps - adjusted_point['y'])**2)

        # Check if GPS coordinates are within the specified radius of the point
        if distance < radius:
            # Check if the point has not been passed recently
            if passed_point is None or adjusted_point != passed_point['point']:
                passed_point = {'point': adjusted_point, 'distance': distance}
                timestamp = gps_data.header.stamp

                print("Adjusted X: {:.6f}".format(adjusted_point['x']))
                print("Adjusted Y: {:.6f}".format(adjusted_point['y']))
                print("Timestamp: {}".format(timestamp))
                print("\n")

                # You may want to add additional logic or processing here

            break  # Exit the loop once a matching point is found


    return passed_point

def gps_listener(json_data_map):
    rospy.init_node('gps_listener', anonymous=True)

    # Variable to track the point that has been passed
    passed_point = None

    rospy.Subscriber('tric_navigation/gps/head_data', NavSatFix, lambda gps_data: gps_callback(gps_data, json_data_map, passed_point))
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

