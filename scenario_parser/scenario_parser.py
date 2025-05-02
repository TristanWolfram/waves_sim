import random

import parsing_methods as pm

# input:
STATIC = "cliff-statics"
DYNAMIC = "cliff-dynamics"

# Parsing of dynamic data:

# Wave parameters
INCLUDE_WAVE_NOISE = True

# example definition:
# "Boat0": {
#     "lidar": "ouster_os1",
#     "camera": {"specs": "cinematic_cam", "pos": "-4.2 1.75 0.0", "rpy": "0.0 1.57 3.14"},
#     "gps": "gps_base",
#     "imu": "imu_clear",
# },

VEHICLES_WITH_SENSORS = {
    "Boat0": {
        "camera": {"specs": "zed", "pos": "-0.425 0.154 -0.297", "rpy": "0.0 1.57 3.14"},
        "lidar": {"specs": "ouster_os1", "pos": "0.0 0.45 0.0"},
        "gps": {"specs": "gps_base", "pos": "0.0 0.0 0.0"},
        "imu": {"specs": "imu_base", "pos": "0.0 0.0 0.0"},
    }, # BlueBoat
}

LIDAR_PARAMETERS = {
    "ouster_os1": {
        "min_range": "0.5",
        "max_range": "170.0",
        "rate": "10.0",
        "noise": "0.0",
    },
    "small_lidar": {
        "min_range": "0.5",
        "max_range": "50.0",
        "rate": "10.0",
        "noise": "0.0001",
    },
}

CAMERA_PARAMETERS = {
    "cinematic_cam": {"res_x": "1920", "res_y": "1080", "fov": "90.0", "rate": "60.0"},
    "FLIR": {"res_x": "640", "res_y": "480", "fov": "110.0", "rate": "10.0"},
    "zed": {"res_x": "960", "res_y": "600", "fov": "110.0", "rate": "30.0"},
}

GPS_PARAMETERS = {
    "gps_base": {"rate": "1.0", "noise": "0.5"},
}

IMU_PARAMETERS = {
    "imu_base": {
        "rate": "1.0",
        "range": {"ang_vel": "10.0 10.0 5.0", "lin_acc": "10.0"},
        "noise": {
            "angle": "0.01 0.01 0.01",
            "ang_vel": "0.05",
            "yaw_drift": "0.001",
            "lin_acc": "0.1",
        },
    },
    "imu_clear": {
        "rate": "1.0",
        "range": {"ang_vel": "10.0 10.0 5.0", "lin_acc": "10.0"},
    },
}

VEHICLE_MODELS = {
    "Boat0": {
        "model": "boats/blueboat.obj",
        "phys_model": "boats/blueboat_phys.obj",
        "material": "Boat",
        "look": "Blue",
        "depth": -0.25,
    },
    "Boat1": {
        "model": "boats/fisher_boat.obj",
        "phys_model": "boats/fisher_boat_cube_phys.obj",
        "material": "Boat",
        "look": "Fisherboat",
        "depth": 0.5,
    },
    "Boat2": {
        "model": "boats/fisher_boat.obj",
        "phys_model": "boats/fisher_boat_cube_phys.obj",
        "material": "Boat",
        "look": "Fisherboat",
        "depth": 0.5,
    },
    "Boat3": {
        "model": "boats/fisher_boat.obj",
        "phys_model": "boats/fisher_boat_cube_phys.obj",
        "material": "Boat",
        "look": "Fisherboat",
        "depth": 0.5,
    },
    "Boat4": {
        "model": "boats/old_boat.obj",
        "phys_model": "boats/old_boat_cube_phys.obj",
        "material": "Boat",
        "look": "old_boat",
        "depth": 0.4,
    },
    "Boat5": {
        "model": "boats/old_boat.obj",
        "phys_model": "boats/old_boat_cube_phys.obj",
        "material": "Boat",
        "look": "old_boat",
        "depth": 0.4,
    },
    "Boat6": {        
        "model": "boats/old_boat.obj",
        "phys_model": "boats/old_boat_cube_phys.obj",
        "material": "Boat",
        "look": "old_boat",
        "depth": 0.4,},
    "Boat7": {"model": "boats/res_boat.obj", "phys_model": "boats/res_boat_cube_phys.obj", "material": "Boat", "look": "res_boat", "depth": 0.9},
    "Boat8": {"model": "boats/res_boat.obj", "phys_model": "boats/res_boat_cube_phys.obj", "material": "Boat", "look": "res_boat", "depth": 0.9},
    "Boat9": {"model": "boats/res_boat.obj", "phys_model": "boats/res_boat_cube_phys.obj", "material": "Boat", "look": "res_boat", "depth": 0.9},
}

json_input = f"scenario_parser/{DYNAMIC}.json"
output_file = f"metadata/{DYNAMIC}.scn"
pm.parse_dynamic_to_xml(
    json_input,
    output_file,
    VEHICLES_WITH_SENSORS,
    VEHICLE_MODELS,
    LIDAR_PARAMETERS,
    CAMERA_PARAMETERS,
    GPS_PARAMETERS,
    IMU_PARAMETERS,
    INCLUDE_WAVE_NOISE,
)

# Parsing of static data:

# list for model replacements
# .obj file must be located in data/obstacles
# the file ending <.obj> is not needed in the model name
# rpy is the rotation of the model
MODEL_REPLACEMENTS = {
    "r1": {
        "model": "r1",
        "phys_model": "r1_phys",
        "look": "r1",
        "rpy": (-1.57, 0.0, 0.0),
    },
    "r2": {
        "model": "r2",
        "phys_model": "r2_phys",
        "look": "r2",
        "rpy": (-1.57, 0.0, 0.0),
    },
    "r3": {
        "model": "r3",
        "phys_model": "r3_phys",
        "look": "r3",
        "rpy": (-1.57, 0.0, 0.0),
    },
    "r4": {
        "model": "r4",
        "phys_model": "r4_phys",
        "look": "r4",
        "rpy": (1.57, 0.0, 0.0),
    },
    "r5": {
        "model": "r5",
        "phys_model": "r5_phys",
        "look": "r5",
        "rpy": (-1.57, 0.0, 0.0),
    },
    "r6": {
        "model": "r6",
        "phys_model": "r6_phys",
        "look": "r6",
        "rpy": (-1.57, 0.0, 0.0),
    },
}

y_rotations = [random.uniform(0, 6.28) for _ in range(3)]

for i in range(1, 22):
    model_choice = random.choice(["windmill1", "windmill2", "windmill3"])
    group_index = (i - 1) // 7  # Determines which group (0-6, 7-13, 14-20)
    MODEL_REPLACEMENTS[f"wm{i}"] = {
        "model": model_choice,
        "phys_model": "windmill_phys",
        "look": "windmill",
        "rpy": (
            -1.57,
            0.0,
            y_rotations[group_index],
        ),  # Assign the same y rotation within each group
    }

CREATE_OUTER_BORDER = False

json_input_statics = f"scenario_parser/{STATIC}.json"
output_file_statics = f"metadata/{STATIC}.scn"
pm.parse_static_to_xml(
    json_input_statics, output_file_statics, CREATE_OUTER_BORDER, MODEL_REPLACEMENTS
)
