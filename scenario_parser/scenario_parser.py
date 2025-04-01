import parsing_methods as pm
import random

# input:
STATIC = "02-statics"
DYNAMIC = "02-dynamics"

# Parsing of dynamic data:

# Wave parameters
INCLUDE_WAVE_NOISE = True

VEHICLES_WITH_SENSORS = {
    "Boat0": {"lidar": "ouster_os1", "camera": "zed", "gps": "gps_base", "imu": "imu_clear"},
    }

LIDAR_PARAMETERS = {
    "ouster_os1": {"min_range": "0.5", "max_range": "170.0", "rate": "10.0", "noise": "0.0"},
    "small_lidar": {"min_range": "0.5", "max_range": "50.0", "rate": "10.0", "noise": "0.0001"},
}

CAMERA_PARAMETERS = {
    "base_cam": {"res_x": "1280", "res_y": "720", "fov": "90.0", "rate": "10.0"},
    "FLIR": {"res_x": "640", "res_y": "480", "fov": "110.0", "rate": "10.0"},
    "zed": {"res_x": "960", "res_y": "600", "fov": "110.0", "rate": "15.0"},
}

GPS_PARAMETERS = {
    "gps_base": {"rate": "1.0", "noise": "0.5"},
}

IMU_PARAMETERS = {
    "imu_base": {"rate": "1.0", 
                 "range": {"ang_vel": "10.0 10.0 5.0", "lin_acc": "10.0"},
                 "noise": {"angle": "0.01 0.01 0.01", "ang_vel":"0.05", "yaw_drift": "0.001", "lin_acc": "0.1"}
                },
    "imu_clear": {"rate": "1.0",
                  "range": {"ang_vel": "10.0 10.0 5.0", "lin_acc": "10.0"}
                },
                  
}

VEHICLE_MODELS = {
    "Boat0": {"model": "boats/fisher_boat.obj", "phys_model": "boats/fisher_boat_cube_phys.obj", "material": "Boat", "look": "Fisherboat", "depth": 0.5},
    "Boat1": {"model": "boats/fisher_boat.obj", "phys_model": "boats/fisher_boat_cube_phys.obj", "material": "Boat", "look": "Fisherboat", "depth": 0.5},
    "Boat2": {"model": "boats/fisher_boat.obj", "phys_model": "boats/fisher_boat_cube_phys.obj", "material": "Boat", "look": "Fisherboat", "depth": 0.5},
    "Boat3": {"model": "boats/old_boat.obj", "phys_model": "boats/old_boat_cube_phys.obj", "material": "Boat", "look": "old_boat", "depth": 0.4},
    "Boat4": {"model": "boats/old_boat.obj", "phys_model": "boats/old_boat_cube_phys.obj", "material": "Boat", "look": "old_boat", "depth": 0.4},
    "Boat5": {"model": "boats/old_boat.obj", "phys_model": "boats/old_boat_cube_phys.obj", "material": "Boat", "look": "old_boat", "depth": 0.4},
    # "Boat6": {"model": "boats/old_boat.obj", "phys_model": "boats/old_boat_cube_phys.obj", "material": "Boat", "look": "old_boat", "depth": 0.4},
    # "Boat7": {"model": "boats/old_boat.obj", "phys_model": "boats/old_boat_cube_phys.obj", "material": "Boat", "look": "old_boat", "depth": 0.4},
    # "Boat8": {"model": "boats/old_boat.obj", "phys_model": "boats/old_boat_cube_phys.obj", "material": "Boat", "look": "old_boat", "depth": 0.2},
}

# "Boat6": {"model": "boats/res_boat.obj", "phys_model": "boats/res_boat_cube_phys.obj", "material": "Boat", "look": "LightGray", "depth": 0.9},
# "Boat7": {"model": "boats/res_boat.obj", "phys_model": "boats/res_boat_cube_phys.obj", "material": "Boat", "look": "LightGray", "depth": 0.9},
# "Boat8": {"model": "boats/res_boat.obj", "phys_model": "boats/res_boat_cube_phys.obj", "material": "Boat", "look": "LightGray", "depth": 0.9},

json_input = f"scenario_parser/{DYNAMIC}.json"
output_file = f"metadata/{DYNAMIC}.scn"
pm.parse_dynamic_to_xml(json_input,
                        output_file,
                        VEHICLES_WITH_SENSORS,
                        VEHICLE_MODELS,
                        LIDAR_PARAMETERS,
                        CAMERA_PARAMETERS,
                        GPS_PARAMETERS,
                        IMU_PARAMETERS,
                        INCLUDE_WAVE_NOISE,)

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
