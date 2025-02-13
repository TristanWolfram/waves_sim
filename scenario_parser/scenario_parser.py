import parsing_methods as pm

# Parsing of dynamic data:

# Wave parameters
INCLUDE_WAVE_NOISE = True
wave_amplitude = 0.005
wave_frequency = 3.0

VEHICLES_WITH_CAMERA = [
    "Boat3"
    ]

json_input = "scenario_parser/dynamics.json"
output_file = "metadata/dynamics.scn"
pm.parse_dynamic_to_xml(json_input, output_file, VEHICLES_WITH_CAMERA, INCLUDE_WAVE_NOISE, wave_amplitude, wave_frequency)

# Parsing of static data:

# list for model replacements 
# name should correspont to both the .obj file and the .png / .jpg (texture) file
# .obj file must be located in data/obstacles
MODEL_REPLACEMENTS = {
    "Pier1": {"model": "rocks_line2", "rpy": (-1.57, 0.0, 0.0)},
    "Pier2": {"model": "rocks_line", "rpy": (-1.57, 0.0, 0.0)},
    "Building1": {"model": "rock3", "rpy": (-1.57, 0.0, 0.0)},
    "Building2": {"model": "stone_formation", "rpy": (1.57, 0.0, 0.0)},
    "Building3": {"model": "rock3", "rpy": (-1.57, 0.0, 0.0)},
}

CREATE_OUTER_BORDER = False

json_input_statics = "scenario_parser/statics.json"
output_file_statics = "metadata/statics.scn"
pm.parse_static_to_xml(json_input_statics, output_file_statics, CREATE_OUTER_BORDER, MODEL_REPLACEMENTS)
