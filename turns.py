import json
import matplotlib.pyplot as plt

# Function to load JSON data
def load_json(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data

def plot_turns_between_treatment_areas(json_data):
    # Extracting x and y coordinates from the JSON data
    x = [point['head']['position']['x'] + json_data['datum']['longitude'] for point in json_data['points']]
    y = [point['head']['position']['y'] + json_data['datum']['latitude'] for point in json_data['points']]

    # Finding indices of treatment area true points
    treatment_area_indices = [i for i, point in enumerate(json_data['points']) if point['treatment_area']]

    # Extracting turns and adjusting their coordinates
    turns = [
        {
            'x': point['head']['position']['x'] + json_data['datum']['longitude'],
            'y': point['head']['position']['y'] + json_data['datum']['latitude']
        }
        for i in range(1, len(treatment_area_indices))  # Iterate from the second treatment area point
        for point in json_data['points'][treatment_area_indices[i - 1] + 1 : treatment_area_indices[i]]
    ]

    # Extracting x and y coordinates from the adjusted turns
    turns_x = [point['x'] for point in turns]
    turns_y = [point['y'] for point in turns]

    # Creating a 2D scatter plot for the turns
    plt.scatter(turns_x, turns_y, color='orange', label='Turns Between Treatment Areas')

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

# Plot turns only between two treatment area true points
plot_turns_between_treatment_areas(json_data)
