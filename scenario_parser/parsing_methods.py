import json
import xml.etree.ElementTree as ET
import math
import random


def parse_dynamic_to_xml(json_file, output_file, vehicles_with_camera, vehicle_models, INCLUDE_WAVE_NOISE=False, wave_amplitude=0.1, wave_frequency=0.5):
    with open(json_file, "r") as f:
        json_data = json.load(f)

    root = ET.Element("scenario")
    vehicles = {}
    number_of_vehicles_with_camera = 0

    for time_key, vehicle_data in json_data["time_s"].items():
        time_float = float(time_key)  # Ensure time is a float

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
                ET.SubElement(
                    physical, "mesh", filename=phys_model, scale="1.0"
                )
                ET.SubElement(physical, "origin", xyz="0.0 0.0 0.0", rpy="0.0 0.0 0.0")

                # Visual element
                visual = ET.SubElement(animated, "visual")
                ET.SubElement(
                    visual, "mesh", filename=model, scale="1.0"
                )
                ET.SubElement(visual, "origin", xyz="0.0 0.0 0.0", rpy="0.0 0.0 0.0")

                # Material and Look
                ET.SubElement(animated, "material", name=material)
                ET.SubElement(animated, "look", name=look, uv_mode="0")

                # Trajectory element
                trajectory = ET.SubElement(
                    animated, "trajectory", type="spline", playback="repeat"
                )
                vehicles[vehicle_name] = trajectory

                if vehicle_name in vehicles_with_camera:

                    print(f"Adding camera to {vehicle_name}, ROS2 topic -> /sim_cam_color{number_of_vehicles_with_camera}")
                    print(f"Adding depth camera to {vehicle_name}, ROS2 topic -> /sim_cam_depth{number_of_vehicles_with_camera}")

                    sensor = ET.SubElement(animated, "sensor", name="Cam", rate="10.0", type="camera")
                    ET.SubElement(sensor, "specs", resolution_x="1280", resolution_y="720", horizontal_fov="90.0")
                    ET.SubElement(sensor, "noise", depth="0.02")
                    ET.SubElement(sensor, "origin", xyz="-3.2 1.75 0.0", rpy="0.0 1.57 3.14")
                    ET.SubElement(sensor, "ros_publisher", topic=f"/sim_cam_color{number_of_vehicles_with_camera}")  

                    cameras = [
                        {"name": "DcamF", "xyz": "-1.0 3.0 0.0", "rpy": "0.0 1.57 3.14", "topic": "/sim_camF_depth"},
                        {"name": "DcamR", "xyz": "-1.0 3.0 0.0", "rpy": "0.0 3.14 3.14", "topic": "/sim_camR_depth"},
                        {"name": "DcamL", "xyz": "-1.0 3.0 0.0", "rpy": "0.0 0.0 3.14", "topic": "/sim_camL_depth"},
                        {"name": "DcamB", "xyz": "-1.0 3.0 0.0", "rpy": "0.0 -1.57 3.14", "topic": "/sim_camB_depth"},
                    ]

                    for cam in cameras:
                        sensor = ET.SubElement(animated, "sensor", name=cam["name"], rate="10.0", type="depthcamera")
                        ET.SubElement(sensor, "specs", resolution_x="256", resolution_y="128", horizontal_fov="90.0", depth_min="0.2", depth_max="100.0")
                        ET.SubElement(sensor, "noise", depth="0.02")
                        ET.SubElement(sensor, "origin", xyz=cam["xyz"], rpy=cam["rpy"])
                        ET.SubElement(sensor, "ros_publisher", topic=f"{cam["topic"]}_{number_of_vehicles_with_camera}")

                    number_of_vehicles_with_camera += 1                 

            # Add keypoints to the trajectory
            trajectory = vehicles[vehicle_name]
            position = details["center_position_m"]
            heading = details["heading_rad"] + 3.14159  # Adjust heading by 180 degrees

            z = 0.5  # Default depth of the boat
            pitch = -1.57  # Default pitch of the boat
            roll = 0.0  # Default roll of the boat

            if INCLUDE_WAVE_NOISE:

                # ** Add wave motion **
                # 0.5 is the base depth of the boat (stonefish reverses the z-axis)
                z += wave_amplitude * math.sin(wave_frequency * time_float)

                # ** Add noise to the pitch and roll **
                pitch_amplitude = 0.015
                roll_amplitude = 0.015  
                phase_shift_pitch = math.pi / 4
                phase_shift_roll = math.pi / 2   


                pitch += pitch_amplitude * math.sin(wave_frequency * time_float + phase_shift_pitch)
                roll += roll_amplitude * math.sin(wave_frequency * time_float + phase_shift_roll)

            ET.SubElement(
                trajectory,
                "keypoint",
                time=str(time_key),
                xyz=f"{position[0]} {position[1]} {z}",
                rpy=f"{pitch} {roll} {heading}"
            )

    # Generate the XML tree and write it to a file
    tree = ET.ElementTree(root)
    ET.indent(tree, space="    ", level=0)  # Pretty print the XML
    tree.write(output_file, encoding="utf-8", xml_declaration=True)
    print(f"Dynamic XML file has been generated: {output_file}")


def parse_static_to_xml(json_file, output_file, CREATE_OUTER_BORDER=False, MODEL_REPLACEMENTS={}):
    # Read the JSON file
    with open(json_file, 'r') as f:
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
                    length = ((end[0] - start[0])**2 + (end[1] - start[1])**2)**0.5
                    center_x = (start[0] + end[0]) / 2
                    center_y = (start[1] + end[1]) / 2
                    
                    # Compute angle using atan2 for correct orientation
                    angle = math.atan2(end[1] - start[1], end[0] - start[0])  

                    static = ET.SubElement(root, "static", name=f"{wall_name}_side{i}", type="box")
                    ET.SubElement(static, "dimensions", xyz=f"{length} 0.1 1.0")
                    ET.SubElement(static, "material", name="Steel")
                    ET.SubElement(static, "look", name="Gray")
                    ET.SubElement(static, "world_transform", xyz=f"{center_x} {center_y} 0.0", rpy=f"0.0 0.0 {angle}")

                    print(f"Found outer border, drawing at {center_x} {center_y} with length {length} and angle {angle}")

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
                    ET.SubElement(physical, "mesh", filename=f"obstacles/{phys_file}.obj", scale="1.0")
                    ET.SubElement(physical, "origin", xyz="0.0 0.0 0.0", rpy="-1.57 0.0 0.0")

                    visual = ET.SubElement(static, "visual")
                    ET.SubElement(visual, "mesh", filename=f"obstacles/{model_file}.obj", scale="1.0")
                    ET.SubElement(visual, "origin", xyz="0.0 0.0 0.0", rpy=f"{rpy_values[0]} {rpy_values[1]} {rpy_values[2]}")

                    # hard coded for now
                    ET.SubElement(static, "material", name="Stone")
                    ET.SubElement(static, "look", name=look)
                    ET.SubElement(static, "world_transform", xyz=f"{center_x} {center_y} 0.5", rpy="0.0 0.0 0.0")
                else:
                    static = ET.SubElement(root, "static", name=wall_name, type="box")
                    ET.SubElement(static, "dimensions", xyz=f"{length} {width} {height}")
                    ET.SubElement(static, "material", name="Steel")
                    ET.SubElement(static, "look", name="Gray")
                    ET.SubElement(static, "world_transform", xyz=f"{center_x} {center_y} {(-height / 2) + 1}", rpy="0.0 0.0 0.0")
                    # +1 to place the wall inside the water

    tree = ET.ElementTree(root)
    ET.indent(tree, space="    ", level=0)
    tree.write(output_file, encoding="utf-8", xml_declaration=True)
    print(f"Static XML file has been generated: {output_file}")