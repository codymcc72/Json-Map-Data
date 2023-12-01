#!/Library/Frameworks/Python.framework/Versions/3.12/bin/python3

'''

JsonDataMap Class:

The JsonDataMap class is responsible for extracting relevant information from the JSON data and visualizing it using the Matplotlib library. 

1. Data Extraction:
   - The class has methods to extract rows, turns, start paths, end paths, and the datum from the given JSON data. These methods adjust the coordinates of the points based on the map's datum.
   - Rows represent treatment areas, and the extracted data includes x and y coordinates.

2. Plotting Data:
   - The `plot_data` method utilizes Matplotlib to create a scatter plot of the extracted data. Different colors are used for rows, turns, start paths, end paths, and the datum.
   - Labels, legend, and axes information are added to enhance the clarity of the plot.

IdealTime Class:

The `IdealTime` class focuses on calculating and storing ideal travel times based on the extracted rows. Here's a detailed explanation of its functionalities:

1. Initialization:
   - The class is initialized with JSON data and a speed parameter, representing the speed of travel in meters per second.

2. Distance Calculation:
   - The `calculate_distance` method calculates the Euclidean distance between two points. This is used to determine the total distance traveled in each row.

3. Travel Time Calculation:
   - The `calculate_and_store_travel_times` method processes the rows from the JSON data and calculates the total distance traveled in each row.
   - The total distance is then divided by the specified speed to obtain the travel time in seconds.
   - The calculated travel times are stored in the `self.ideal_travel_times` list.


'''

import json
import math
import matplotlib.pyplot as plt

class JsonDataMap:
    def __init__(self):
        self.rows = None
        self.turns = None
        self.start_path = None
        self.end_path = None
        self.datum = None

    def extract_rows(self, json_data):
        x = [point['head']['position']['x'] + json_data['datum']['longitude'] for point in json_data['points']]
        y = [point['head']['position']['y'] + json_data['datum']['latitude'] for point in json_data['points']]

        # Extracting row points and adjusting their coordinates
        rows = [
            {
                'x': point['head']['position']['x'] + json_data['datum']['longitude'],
                'y': point['head']['position']['y'] + json_data['datum']['latitude']
            }
            for point in json_data['points'] if point.get('treatment_area', False)
        ]

        # Extracting x and y coordinates from the adjusted rows
        rows_x = [point['x'] for point in rows]
        rows_y = [point['y'] for point in rows]

        return {'x': rows_x, 'y': rows_y}
    
    def extract_turns(self, json_data):
        x = [point['head']['position']['x'] + json_data['datum']['longitude'] for point in json_data['points']]
        y = [point['head']['position']['y'] + json_data['datum']['latitude'] for point in json_data['points']]

        # Finding indices of treatment area true points
        treatment_area_indices = [i for i, point in enumerate(json_data['points']) if point.get('treatment_area', False)]

        # Adjusting the turn coordinates coordinates
        turns = [
            {
                'x': point['head']['position']['x'] + json_data['datum']['longitude'],
                'y': point['head']['position']['y'] + json_data['datum']['latitude']
            }
            for i in range(1, len(treatment_area_indices))  # Iterating from the second treatment area point
            for point in json_data['points'][treatment_area_indices[i - 1] + 1: treatment_area_indices[i]]
        ]

        turns_x = [point['x'] for point in turns]
        turns_y = [point['y'] for point in turns]

        return {'x': turns_x, 'y': turns_y}

    def extract_start_path(self, json_data):
        x = [point['head']['position']['x'] + json_data['datum']['longitude'] for point in json_data['points']]
        y = [point['head']['position']['y'] + json_data['datum']['latitude'] for point in json_data['points']]

        # Finding the index of the first treatment area point
        first_treatment_area_index = next((i for i, point in enumerate(json_data['points']) if point.get('treatment_area', False)), None)

        # Defining the range for the start path
        start_path_x = x[:first_treatment_area_index]
        start_path_y = y[:first_treatment_area_index]

        return {'x': start_path_x, 'y': start_path_y}

    def extract_end_path(self, json_data):
        x = [point['head']['position']['x'] + json_data['datum']['longitude'] for point in json_data['points']]
        y = [point['head']['position']['y'] + json_data['datum']['latitude'] for point in json_data['points']]

        # Finding the index of the last point in the map
        last_point_index = len(json_data['points']) - 1

        # Finding the index of the last treatment area point
        last_treatment_area_index = next((i for i in range(last_point_index, -1, -1) if json_data['points'][i].get('treatment_area', False)), None)

        # Defining the x and y range for the end path
        end_path_x = x[last_treatment_area_index:]
        end_path_y = y[last_treatment_area_index:]

        return {'x': end_path_x, 'y': end_path_y}

    def extract_datum(self, json):
        x = json['datum']['longitude']
        y = json['datum']['latitude']
        return {'x': x, 'y': y}

    def plot_data(self):
        #rows
        plt.scatter(self.rows['x'], 
                    self.rows['y'], 
                    color='mediumseagreen', 
                    label='Rows', 
                    s=10)
        
        #turns
        plt.scatter(self.turns['x'], 
                    self.turns['y'], 
                    color='lightsteelblue', 
                    label='Turns', 
                    s=10)
        
        #start path
        plt.scatter(self.start_path['x'], 
                    self.start_path['y'], 
                    color='steelblue', 
                    label='Start Path', 
                    s=10)
        
        #end path
        plt.scatter(self.end_path['x'], 
                    self.end_path['y'], 
                    color='tomato', 
                    label='End Path', 
                    s=10)

        # Plotting the datum
        plt.scatter(self.datum['x'], 
                    self.datum['y'], 
                    color='black', 
                    marker='x', 
                    label='Home')

        # Adding labels
       

        plt.xlabel('X Label')
        plt.ylabel('Y Label')

        # Adding a legend
        plt.legend()

        # Show the plot
        plt.show()


class IdealTime:
    def __init__(self, json_data, speed):
        self.json_data = json_data
        self.speed = speed
        self.ideal_travel_times = []  # Add a list to store ideal travel times

    def calculate_distance(self, point1, point2):
        x1, y1, z1 = point1['head']['position']['x'], point1['head']['position']['y'], point1['head']['position']['z']
        x2, y2, z2 = point2['head']['position']['x'], point2['head']['position']['y'], point2['head']['position']['z']
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
        return distance

    def calculate_and_store_travel_times(self):
        # Extracting relevant information
        points = self.json_data['points']
        rows = []

        # Flag to indicate if a row is currently being processed
        in_row = False

        # Temporary storage for the current row
        current_row = []

        # Iterate through points
        for point in points:
            if point['treatment_area']:
                # Treatment area is true
                if not in_row:
                    # Start of a new row
                    in_row = True
                    current_row = [point]
                else:
                    # Continue adding points to the current row
                    current_row.append(point)
            elif in_row:
                # Treatment area is false, and we were in a row
                in_row = False
                # Check if the row has more than one point (turn)
                if len(current_row) > 1:
                    rows.append(current_row)

        # Calculate and store travel time for each row
        for i, row in enumerate(rows):
            total_distance = 0
            for j in range(len(row) - 1):
                distance = self.calculate_distance(row[j], row[j + 1])
                total_distance += distance

            travel_time_seconds = total_distance / self.speed
            travel_time_minutes = travel_time_seconds / 60  # Convert seconds to minutes

            # Store the calculated travel time
            self.ideal_travel_times.append(travel_time_minutes)

            print(f"Row {i + 1}: Ideal Travel Time = {travel_time_minutes:.2f} minutes")

    def get_ideal_travel_times(self):
        return self.ideal_travel_times

def main():
    # Load JSON data
    with open('appended.json', 'r') as file:
        json_data = json.load(file)

    # Set the speed for the ideal time calculation (in meters per second)
    speed = 0.23  # Adjust this value as needed

    # Create an instance of the JsonDataMap class
    plot_data = JsonDataMap()

    # Create an instance of the IdealTime class with the specified speed
    ideal_time = IdealTime(json_data, speed)

    # Extract and store data
    plot_data.rows = plot_data.extract_rows(json_data)
    plot_data.turns = plot_data.extract_turns(json_data)
    plot_data.start_path = plot_data.extract_start_path(json_data)
    plot_data.end_path = plot_data.extract_end_path(json_data)
    plot_data.datum = plot_data.extract_datum(json_data)

    # Plot the data
    plot_data.plot_data()

    # Calculate, print, and store ideal travel times
    ideal_time.calculate_and_store_travel_times()

    # Access the stored ideal travel times for future use
    stored_ideal_times = ideal_time.get_ideal_travel_times()
    print("Stored Ideal Travel Times:", stored_ideal_times)

if __name__ == "__main__":
    main()
