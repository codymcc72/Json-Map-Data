import json
import matplotlib.pyplot as plt

# Function to load JSON data
def load_json(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data

def plot_start_path(json_data):
    # Extracting x and y coordinates from the JSON data
    x = [point['head']['position']['x'] + json_data['datum']['longitude'] for point in json_data['points']]
    y = [point['head']['position']['y'] + json_data['datum']['latitude'] for point in json_data['points']]

    # Finding the index of the first treatment area point
    first_treatment_area_index = next((i for i, point in enumerate(json_data['points']) if point['treatment_area']), None)

    # Defining the x and y range for the start path
    start_path_x = x[:first_treatment_area_index]
    start_path_y = y[:first_treatment_area_index]

    # Creating a 2D scatter plot for the start path
    plt.scatter(start_path_x, start_path_y, color='blue', label='Start Path')

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

# Plot the start path
plot_start_path(json_data)
