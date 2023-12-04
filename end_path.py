import json
import matplotlib.pyplot as plt

# Function to load JSON data
def load_json(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data

def plot_end_path(json_data):
    # Extracting x and y coordinates from the JSON data
    x = [point['head']['position']['x'] + json_data['datum']['longitude'] for point in json_data['points']]
    y = [point['head']['position']['y'] + json_data['datum']['latitude'] for point in json_data['points']]

    # Finding the index of the last point in the map
    last_point_index = len(json_data['points']) - 1

    # Finding the index of the last treatment area point
    last_treatment_area_index = next((i for i in range(last_point_index, -1, -1) if json_data['points'][i]['treatment_area']), None)

    # Defining the x and y range for the end path
    end_path_x = x[last_treatment_area_index:]
    end_path_y = y[last_treatment_area_index:]

    # Creating a 2D scatter plot for the end path
    plt.scatter(end_path_x, end_path_y, color='red', label='End Path')

    # Plotting the datum
    plt.scatter(json_data['datum']['longitude'], json_data['datum']['latitude'], color='purple', marker='x', label='Datum')

    # Adding labels
    plt.xlabel('X Label')
    plt.ylabel('Y Label')

    # Adding a legend
    plt.legend()

    # Show the plot
    plt.show()

# Load JSON data
json_data = load_json('appended.json')

# Plot the end path
plot_end_path(json_data)
