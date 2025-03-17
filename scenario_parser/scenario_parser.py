import parsing_methods as pm
import random

# input:
STATIC = "01-statics"
DYNAMIC = "01-dynamics"

# Parsing of dynamic data:

# Wave parameters
INCLUDE_WAVE_NOISE = True
wave_amplitude = 0.005
wave_frequency = 3.0

VEHICLES_WITH_CAMERA = [
    "Boat1"
    ]

VEHICLE_MODELS = {
    "Boat0": {"model": "boats/fisher_boat.obj", "phys_model": "boats/fisher_boat_cube_phys.obj", "material": "Boat", "look": "Fisherboat"},
    "Boat1": {"model": "boats/fisher_boat.obj", "phys_model": "boats/fisher_boat_cube_phys.obj", "material": "Boat", "look": "Fisherboat"},
    "Boat2": {"model": "boats/fisher_boat.obj", "phys_model": "boats/fisher_boat_cube_phys.obj", "material": "Boat", "look": "Fisherboat"},
    "Boat3": {"model": "boats/fisher_boat.obj", "phys_model": "boats/fisher_boat_cube_phys.obj", "material": "Boat", "look": "Fisherboat"},
    "Boat4": {"model": "boats/fisher_boat.obj", "phys_model": "boats/fisher_boat_cube_phys.obj", "material": "Boat", "look": "Fisherboat"},
    "Boat5": {"model": "boats/fisher_boat.obj", "phys_model": "boats/fisher_boat_cube_phys.obj", "material": "Boat", "look": "Fisherboat"},
    "Boat6": {"model": "boats/res_boat.obj", "phys_model": "boats/res_boat_cube_phys.obj", "material": "Boat", "look": "LightGray"},
    "Boat7": {"model": "boats/res_boat.obj", "phys_model": "boats/res_boat_cube_phys.obj", "material": "Boat", "look": "LightGray"},
    "Boat8": {"model": "boats/res_boat.obj", "phys_model": "boats/res_boat_cube_phys.obj", "material": "Boat", "look": "LightGray"},
}

json_input = f"scenario_parser/{DYNAMIC}.json"
output_file = f"metadata/{DYNAMIC}.scn"
pm.parse_dynamic_to_xml(json_input, output_file, VEHICLES_WITH_CAMERA, VEHICLE_MODELS, INCLUDE_WAVE_NOISE, wave_amplitude, wave_frequency)

# Parsing of static data:

# list for model replacements 
# .obj file must be located in data/obstacles
# the file ending <.obj> is not needed in the model name
# rpy is the rotation of the model
MODEL_REPLACEMENTS = {
    "Pier1": {"model": "rocks_line2", "phys_model": "rocks_line2_phys", "look": "rocks_line2", "rpy": (-1.57, 0.0, 0.0)},
    "Pier2": {"model": "rocks_line", "phys_model": "rocks_line_phys", "look": "rocks_line", "rpy": (-1.57, 0.0, 0.0)},
    "Building1": {"model": "rock3", "phys_model": "rock3_phys", "look": "rock3", "rpy": (-1.57, 0.0, 0.0)},
    "Building2": {"model": "stone_formation", "phys_model": "stone_formation_phys", "look": "stone_formation", "rpy": (1.57, 0.0, 0.0)},
    "Building3": {"model": "rock3", "phys_model": "rock3_phys", "look": "rock3", "rpy": (-1.57, 0.0, 0.0)},
}

y_rotations = [random.uniform(0, 6.28) for _ in range(3)]

for i in range(1, 22):
    model_choice = random.choice(["windmill1", "windmill2", "windmill3"])
    group_index = (i - 1) // 7  # Determines which group (0-6, 7-13, 14-20)
    MODEL_REPLACEMENTS[f"wm{i}"] = {
        "model": model_choice,
        "phys_model": "windmill_phys",
        "look": "White",
        "rpy": (-1.57, 0.0, y_rotations[group_index])  # Assign the same y rotation within each group
    }

CREATE_OUTER_BORDER = False

json_input_statics = f"scenario_parser/{STATIC}.json"
output_file_statics = f"metadata/{STATIC}.scn"
pm.parse_static_to_xml(json_input_statics, output_file_statics, CREATE_OUTER_BORDER, MODEL_REPLACEMENTS)
