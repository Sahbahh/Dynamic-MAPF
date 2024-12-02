# input_parser.py

import json

def parse_input(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    
    map_dimensions = data["map_dimensions"]
    agents = data["agents"]
    timesteps = data["timesteps"]
    
    return map_dimensions, agents, timesteps
