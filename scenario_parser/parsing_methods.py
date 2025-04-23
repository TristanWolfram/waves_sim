import json
import math
import random
import xml.etree.ElementTree as ET

import numpy as np

# Parameters for wave noise model
wave_amp_z = 0.006  # Amplitude of wave motion in Z direction
wave_amp_pitch = 0.0018  # Amplitude of wave motion in pitch direction
wave_amp_roll = 0.003  # Amplitude of wave motion in roll direction

noise_scale = 0.0206  # Scale of noise

wave_frequency = 0.05  # Wave frequency in Hz

dt = 0.1  # Time step in seconds
r = 0.998  # Weak damping factor
omega_d = 2 * math.pi * wave_frequency

a = 2 * r * math.cos(omega_d * dt)
b = -(r**2)

# Initialize noise variables
z_noise = [np.random.randn(), np.random.randn()]
pitch_noise = [np.random.randn(), np.random.randn()]
roll_noise = [np.random.randn(), np.random.randn()]


def update_noise(noise_list, a, b, amplitude):
    e = noise_scale * np.random.randn()  # Small random input
    new_value = a * noise_list[-1] + b * noise_list[-2] + e
    noise_list.append(new_value)
    return amplitude * new_value  # Scale by wave_amplitude


def parse_dynamic_to_xml(
    json_file,
    output_file,
    vehicles_with_sensor,
    vehicle_models,
    lidar_specs_combined,
    camera_specs_combined,
    gps_specs_compined,
    imu_specs_combined,
    INCLUDE_WAVE_NOISE=False,
):

    print("Parsing dynamic data to XML...\n")

    with open(json_file, "r") as f:
        json_data = json.load(f)

    root = ET.Element("scenario")
    vehicles = {}
    number_of_vehicles_with_camera = 0

    id_for_testing = 0
    for time_key, vehicle_data in json_data["time_s"].items():

        for vehicle_name, details in vehicle_data.items():
            if vehicle_name not in vehicles:

                # Fetch vehicle model details
                model_details = vehicle_models[vehicle_name]
                phys_model = model_details["phys_model"]
                model = model_details["model"]
                material = model_details["material"]
                look = model_details["look"]

                animated = ET.SubElement(
                    root, "animated", name=vehicle_name, type="model", collition="true"
                )

                # Physical element
                physical = ET.SubElement(animated, "physical")
                ET.SubElement(physical, "mesh", filename=phys_model, scale="1.0")
                ET.SubElement(physical, "origin", xyz="0.0 0.0 0.0", rpy="0.0 0.0 0.0")

                # Visual element
                visual = ET.SubElement(animated, "visual")
                ET.SubElement(visual, "mesh", filename=model, scale="1.0")
                ET.SubElement(visual, "origin", xyz="0.0 0.0 0.0", rpy="0.0 0.0 0.0")

                # Material and Look
                ET.SubElement(animated, "material", name=material)
                ET.SubElement(animated, "look", name=look, uv_mode="0")

                # Trajectory element
                trajectory = ET.SubElement(
                    animated, "trajectory", type="spline", playback="repeat"
                )
                vehicles[vehicle_name] = trajectory

                if vehicle_name in vehicles_with_sensor:

                    sensors = vehicles_with_sensor[vehicle_name]
                    camera_specs = camera_specs_combined[sensors["camera"]]
                    lidar_specs = lidar_specs_combined[sensors["lidar"]]

                    print(f"-----------------> {vehicle_name} has sensors!")
                    print(
                        f"Adding camera ({sensors['camera']}), ROS2 topic -> /sim_cam_color{number_of_vehicles_with_camera}"
                    )
                    print(
                        f"Adding LiDAR ({sensors['lidar']}), ROS2 topic -> /sim_cam_depth{number_of_vehicles_with_camera}"
                    )

                    sensor = ET.SubElement(
                        animated,
                        "sensor",
                        name="Cam",
                        rate=camera_specs["rate"],
                        type="camera",
                    )
                    ET.SubElement(
                        sensor,
                        "specs",
                        resolution_x=camera_specs["res_x"],
                        resolution_y=camera_specs["res_y"],
                        horizontal_fov=camera_specs["fov"],
                    )
                    ET.SubElement(
                        sensor, "origin", xyz="-4.2 1.75 0.0", rpy="0.0 1.57 3.14"
                    )
                    ET.SubElement(
                        sensor,
                        "ros_publisher",
                        topic=f"/sim_cam_color_{number_of_vehicles_with_camera}",
                    )

                    cameras = [
                        {
                            "name": "DcamF",
                            "xyz": "-4.2 1.75 0.0",
                            "rpy": "0.0 1.57 3.14",
                            "topic": "/sim_camF_depth",
                        },
                        {
                            "name": "DcamR",
                            "xyz": "-4.2 1.75 0.0",
                            "rpy": "0.0 3.14 3.14",
                            "topic": "/sim_camR_depth",
                        },
                        {
                            "name": "DcamL",
                            "xyz": "-4.2 1.75 0.0",
                            "rpy": "0.0 0.0 3.14",
                            "topic": "/sim_camL_depth",
                        },
                        {
                            "name": "DcamB",
                            "xyz": "-4.2 1.75 0.0",
                            "rpy": "0.0 -1.57 3.14",
                            "topic": "/sim_camB_depth",
                        },
                    ]

                    for cam in cameras:
                        sensor = ET.SubElement(
                            animated,
                            "sensor",
                            name=cam["name"],
                            rate=lidar_specs["rate"],
                            type="depthcamera",
                        )
                        ET.SubElement(
                            sensor,
                            "specs",
                            resolution_x="256",
                            resolution_y="128",
                            horizontal_fov="90.0",
                            depth_min=lidar_specs["min_range"],
                            depth_max=lidar_specs["max_range"],
                        )
                        ET.SubElement(sensor, "noise", depth=lidar_specs["noise"])
                        ET.SubElement(sensor, "origin", xyz=cam["xyz"], rpy=cam["rpy"])
                        topic = cam["topic"]
                        ET.SubElement(
                            sensor,
                            "ros_publisher",
                            topic=f"{topic}_{number_of_vehicles_with_camera}",
                        )

                    if "gps" in sensors:
                        print(
                            f"Adding GPS ({sensors['gps']}), ROS2 topic -> /sim_gps{number_of_vehicles_with_camera}"
                        )
                        gps_specs = gps_specs_compined[sensors["gps"]]
                        gps = ET.SubElement(
                            animated,
                            "sensor",
                            name="GPS",
                            rate=gps_specs["rate"],
                            type="gps",
                        )
                        ET.SubElement(gps, "noise", ned_position=gps_specs["noise"])
                        ET.SubElement(gps, "history", samples="1")
                        ET.SubElement(
                            gps, "origin", xyz="-4.2 1.75 0.0", rpy="0.0 0.0 0.0"
                        )
                        ET.SubElement(
                            gps,
                            "ros_publisher",
                            topic=f"/sim_gps_{number_of_vehicles_with_camera}",
                        )

                    if "imu" in sensors:
                        print(
                            f"Adding IMU ({sensors['imu']}), ROS2 topic -> /sim_imu{number_of_vehicles_with_camera}"
                        )
                        imu_specs = imu_specs_combined[sensors["imu"]]
                        imu = ET.SubElement(
                            animated,
                            "sensor",
                            name="IMU",
                            rate=imu_specs["rate"],
                            type="imu",
                        )
                        ET.SubElement(
                            imu,
                            "range",
                            angular_velocity=imu_specs["range"]["ang_vel"],
                            linear_acceleration=imu_specs["range"]["lin_acc"],
                        )
                        if "noise" in imu_specs:
                            ET.SubElement(
                                imu,
                                "noise",
                                angle=imu_specs["noise"]["angle"],
                                angular_velocity=imu_specs["noise"]["ang_vel"],
                                yaw_drift=imu_specs["noise"]["yaw_drift"],
                                linear_acceleration=imu_specs["noise"]["lin_acc"],
                            )
                        ET.SubElement(imu, "history", samples="1")
                        ET.SubElement(
                            imu, "origin", xyz="0.0 0.0 0.0", rpy="-1.57 0.0 0.0"
                        )
                        ET.SubElement(
                            imu,
                            "ros_publisher",
                            topic=f"/sim_imu_{number_of_vehicles_with_camera}",
                        )

                    number_of_vehicles_with_camera += 1
                    print("\n")

            trajectory = vehicles[vehicle_name]
            position = details["center_position_m"]
            heading = round(details["heading_rad"] + 3.14159, 3)

            z = vehicle_models[vehicle_name]["depth"]  # Base depth of the boat
            pitch = -1.57  # Default pitch
            roll = 0.0  # Default roll

            # if INCLUDE_WAVE_NOISE:

            #     # ** Add wave motion **
            #     # 0.5 is the base depth of the boat (stonefish reverses the z-axis)
            #     z += wave_amplitude * math.sin(wave_frequency * time_float)

            #     # ** Add noise to the pitch and roll **
            #     pitch_amplitude = 0.015
            #     roll_amplitude = 0.015
            #     phase_shift_pitch = math.pi / 4
            #     phase_shift_roll = math.pi / 2

            #     pitch += pitch_amplitude * math.sin(wave_frequency * time_float + phase_shift_pitch)
            #     roll += roll_amplitude * math.sin(wave_frequency * time_float + phase_shift_roll)

            if INCLUDE_WAVE_NOISE:
                # Apply stochastic wave noise
                z += update_noise(z_noise, a, b, wave_amp_z)
                pitch += update_noise(pitch_noise, a, b, wave_amp_pitch)
                roll += update_noise(roll_noise, a, b, wave_amp_roll)

            z = round(z, 3)
            pitch = round(pitch, 3)
            roll = round(roll, 3)

            ET.SubElement(
                trajectory,
                "keypoint",
                time=str(time_key),
                xyz=f"{position[0]} {position[1]} {z}",
                rpy=f"{pitch} {roll} {heading}",
            )

    # Generate the XML tree and write it to a file
    tree = ET.ElementTree(root)
    ET.indent(tree, space="    ", level=0)  # Pretty print the XML
    tree.write(output_file, encoding="utf-8", xml_declaration=True)

    if INCLUDE_WAVE_NOISE:
        print(
            f"Dynamic data has been generated -----> with wave noise: {output_file}\n"
        )
    else:
        print(f"Dynamic data has been generated: {output_file}\n")


def parse_static_to_xml(
    json_file, output_file, CREATE_OUTER_BORDER=False, MODEL_REPLACEMENTS={}
):
    # Read the JSON file
    with open(json_file, "r") as f:
        json_data = json.load(f)

    # Create the root XML element
    root = ET.Element("scenario")

    # Process JSON data and build XML structure for statics
    for wall_name, wall_data in json_data.items():
        if wall_data["type"] == "wall":
            dimensions = wall_data["sides_m"]

            if wall_name == "OuterBorder":

                if not CREATE_OUTER_BORDER:
                    print("Skipping outer border")
                    continue
                # Handle outer boundary as separate walls
                for i, side in enumerate(dimensions):
                    start, end = side
                    length = (
                        (end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2
                    ) ** 0.5
                    center_x = (start[0] + end[0]) / 2
                    center_y = (start[1] + end[1]) / 2

                    # Compute angle using atan2 for correct orientation
                    angle = math.atan2(end[1] - start[1], end[0] - start[0])

                    static = ET.SubElement(
                        root, "static", name=f"{wall_name}_side{i}", type="box"
                    )
                    ET.SubElement(static, "dimensions", xyz=f"{length} 0.1 1.0")
                    ET.SubElement(static, "material", name="Steel")
                    ET.SubElement(static, "look", name="Gray")
                    ET.SubElement(
                        static,
                        "world_transform",
                        xyz=f"{center_x} {center_y} 0.0",
                        rpy=f"0.0 0.0 {angle}",
                    )

                    print(
                        f"Found outer border, drawing at {center_x} {center_y} with length {length} and angle {angle}"
                    )

            else:
                # Calculate width, height, and position from the sides
                x_min = min([point[0] for side in dimensions for point in side])
                x_max = max([point[0] for side in dimensions for point in side])
                y_min = min([point[1] for side in dimensions for point in side])
                y_max = max([point[1] for side in dimensions for point in side])

                length = x_max - x_min
                width = y_max - y_min
                height = 5.0  # Default height for the box

                # Calculate the center position
                center_x = x_min + length / 2
                center_y = y_min + width / 2

                if wall_name in MODEL_REPLACEMENTS:
                    print(f"Found {wall_name} in model_replacement")
                    model_data = MODEL_REPLACEMENTS[wall_name]
                    model_file = model_data["model"]
                    phys_file = model_data["phys_model"]
                    look = model_data["look"]
                    rpy_values = model_data["rpy"]

                    static = ET.SubElement(root, "static", name=wall_name, type="model")

                    physical = ET.SubElement(static, "physical")
                    ET.SubElement(
                        physical,
                        "mesh",
                        filename=f"obstacles/{phys_file}.obj",
                        scale="1.0",
                    )
                    ET.SubElement(
                        physical, "origin", xyz="0.0 0.0 0.0", rpy="-1.57 0.0 0.0"
                    )

                    visual = ET.SubElement(static, "visual")
                    ET.SubElement(
                        visual,
                        "mesh",
                        filename=f"obstacles/{model_file}.obj",
                        scale="1.0",
                    )
                    ET.SubElement(
                        visual,
                        "origin",
                        xyz="0.0 0.0 0.0",
                        rpy=f"{rpy_values[0]} {rpy_values[1]} {rpy_values[2]}",
                    )

                    # hard coded for now
                    ET.SubElement(static, "material", name="Stone")
                    ET.SubElement(static, "look", name=look)
                    ET.SubElement(
                        static,
                        "world_transform",
                        xyz=f"{center_x} {center_y} 0.5",
                        rpy="0.0 0.0 0.0",
                    )
                else:
                    static = ET.SubElement(root, "static", name=wall_name, type="box")
                    ET.SubElement(
                        static, "dimensions", xyz=f"{length} {width} {height}"
                    )
                    ET.SubElement(static, "material", name="Steel")
                    ET.SubElement(static, "look", name="Gray")
                    ET.SubElement(
                        static,
                        "world_transform",
                        xyz=f"{center_x} {center_y} {(-height / 2) + 1}",
                        rpy="0.0 0.0 0.0",
                    )
                    # +1 to place the wall inside the water

    tree = ET.ElementTree(root)
    ET.indent(tree, space="    ", level=0)
    tree.write(output_file, encoding="utf-8", xml_declaration=True)
    print(f"Static XML file has been generated: {output_file}")
