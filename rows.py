import json
import matplotlib.pyplot as plt

# Function to load JSON data
def load_json(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data

def plot_rows(json_data):
    # Extracting x and y coordinates from the JSON data
    x = [point['head']['position']['x'] + json_data['datum']['longitude'] for point in json_data['points']]
    y = [point['head']['position']['y'] + json_data['datum']['latitude'] for point in json_data['points']]

    # Extracting row points and adjusting their coordinates
    rows = [
        {
            'x': point['head']['position']['x'] + json_data['datum']['longitude'],
            'y': point['head']['position']['y'] + json_data['datum']['latitude']
        }
        for point in json_data['points'] if point['treatment_area']
    ]

    # Extracting x and y coordinates from the adjusted rows
    rows_x = [point['x'] for point in rows]
    rows_y = [point['y'] for point in rows]

    # Creating a 2D scatter plot for the rows (treatment area points)
    plt.scatter(rows_x, rows_y, color='green', label='Rows (Treatment Area Points)')

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

# Plot the row points (treatment area points)
plot_rows(json_data)
