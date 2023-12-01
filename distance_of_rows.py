#!/Library/Frameworks/Python.framework/Versions/3.12/bin/python3

import json
import matplotlib.pyplot as plt

def load_json(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data

json_data = load_json('appended.json')

# Extracting x and y coordinates from the JSON data
x = [point['head']['position']['x'] for point in json_data['points']]
y = [point['head']['position']['y'] for point in json_data['points']]
treatment_areas = [point['treatment_area'] for point in json_data['points']]

# Identify the start and end path indices
start_path_index = treatment_areas.index(False)
end_path_index = len(treatment_areas) - 1 - treatment_areas[::-1].index(False)

# Plotting the rows in blue
plt.scatter(x[:start_path_index], y[:start_path_index], marker='o', color='blue', label='Row Points')

# Plotting the start path points in green
plt.scatter(x[start_path_index:end_path_index + 1], y[start_path_index:end_path_index + 1], marker='o', color='green', label='Start Path Points')

# Plotting the turns in orange
plt.scatter(x[end_path_index + 1:], y[end_path_index + 1:], marker='o', color='orange', label='Turn Points')

# Plotting the end path points in red
plt.scatter(x[end_path_index + 1:], y[end_path_index + 1:], marker='o', color='red', label='End Path Points')

# Adding labels
plt.xlabel('X Label')
plt.ylabel('Y Label')

# Show the plot
plt.legend()
plt.show()
