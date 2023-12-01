#!/Library/Frameworks/Python.framework/Versions/3.12/bin/python3

import json
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

def main():


    # Load JSON data
    with open('appended.json', 'r') as file:
        json_data = json.load(file)

    # Create an instance of the JsonDataMap class
    plot_data = JsonDataMap()

    # Extract and store data
    plot_data.rows = plot_data.extract_rows(json_data)
    plot_data.turns = plot_data.extract_turns(json_data)
    plot_data.start_path = plot_data.extract_start_path(json_data)
    plot_data.end_path = plot_data.extract_end_path(json_data)
    plot_data.datum = plot_data.extract_datum(json_data)


    # Plot the data
    plot_data.plot_data()


if __name__ == "__main__":
    main()
