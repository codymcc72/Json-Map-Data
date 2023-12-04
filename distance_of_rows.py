import json
import math

# Function to calculate Euclidean distance between two points
def calculate_distance(point1, point2):
    x1, y1, z1 = point1['head']['position']['x'], point1['head']['position']['y'], point1['head']['position']['z']
    x2, y2, z2 = point2['head']['position']['x'], point2['head']['position']['y'], point2['head']['position']['z']
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
    return distance

# Function to load JSON data
def load_json(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data

json_data = load_json('appended.json')

# Extracting relevant information
points = json_data['points']
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

# Now 'rows' contains a list of rows with treatment areas
# Each row is a list of points from a treatment area to the next

# Calculate distance for each row
for i, row in enumerate(rows):
    total_distance = 0
    for j in range(len(row) - 1):
        distance = calculate_distance(row[j], row[j + 1])
        total_distance += distance
    print(f"Row {i + 1}: Distance = {total_distance}")
