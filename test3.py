#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
import json
import os

class DataProcessor:
    def __init__(self, json_data):
        self.gps_data = {'x': [], 'y': []}
        self.json_points = {'x': [], 'y': []}

        # Extract datum information from JSON
        self.datum = json_data['datum']

        # Adjust and store coordinates of points from JSON data
        for point in json_data['points']:
            adjusted_point = self.adjust_coordinates(point)
            self.json_points['x'].append(adjusted_point['x'])
            self.json_points['y'].append(adjusted_point['y'])

    def adjust_coordinates(self, point):
        adjusted_x = point['head']['position']['x'] + self.datum['longitude']
        adjusted_y = point['head']['position']['y'] + self.datum['latitude']
        return {'x': adjusted_x, 'y': adjusted_y}

    def gps_callback(self, gps_data):
        x_gps = gps_data.longitude
        y_gps = gps_data.latitude

        # Store GPS data
        self.gps_data['x'].append(x_gps)
        self.gps_data['y'].append(y_gps)

        # Check and print points that match
        self.check_and_print_matching_points()

    def has_passed_point(self, gps_point, json_point, radius=2.0):
        distance = ((gps_point['x'] - json_point['x'])**2 + (gps_point['y'] - json_point['y'])**2)**0.5
        return distance < radius

    def check_and_print_matching_points(self):
        # Get the latest GPS data
        latest_gps_point = {'x': self.gps_data['x'][-1], 'y': self.gps_data['y'][-1]}

        # Check and print points that match
        for idx, json_point in enumerate(zip(self.json_points['x'], self.json_points['y'])):
            if self.has_passed_point(latest_gps_point, {'x': json_point[0], 'y': json_point[1]}):
                print(f"Match Found - Point {idx}")

def gps_listener(data_processor):
    rospy.init_node('gps_listener', anonymous=True)
    rospy.Subscriber('tric_navigation/gps/head_data', NavSatFix, data_processor.gps_callback)
    rospy.spin()

if __name__ == '__main__':
    # Get the path to the JSON map file
    script_dir = os.path.dirname(os.path.realpath(__file__))
    map_file_path = os.path.join(script_dir, 'json_map_plot', 'testrow.json')

    # Load JSON data
    with open(map_file_path, 'r') as file:
        json_data = json.load(file)

    # Initialize DataProcessor with the loaded JSON data
    data_processor = DataProcessor(json_data)

    # Initialize GPS listener with the DataProcessor instance
    gps_listener(data_processor)
