import json
import os
import xml.dom.minidom as minidom
import xml.etree.ElementTree as ET


def json_to_xml(json_file, output_file):

    with open(json_file, "r") as f:
        json_data = json.load(f)

    root = ET.Element("scenario")

    vehicles = {}

    for time_key, vehicle_data in json_data["time_s"].items():
        for vehicle_name, details in vehicle_data.items():
            if vehicle_name not in vehicles:
                animated = ET.SubElement(
                    root, "animated", name=vehicle_name, type="model", collition="true"
                )

                # Physical element
                physical = ET.SubElement(animated, "physical")
                ET.SubElement(
                    physical, "mesh", filename="boats/fisher_boat_phys.obj", scale="1.0"
                )
                ET.SubElement(physical, "origin", xyz="0.0 0.0 0.0", rpy="0.0 0.0 0.0")

                # Visual element
                visual = ET.SubElement(animated, "visual")
                ET.SubElement(
                    visual, "mesh", filename="boats/fisher_boat.obj", scale="1.0"
                )
                ET.SubElement(visual, "origin", xyz="0.0 0.0 0.0", rpy="0.0 0.0 0.0")

                # Material and Look
                ET.SubElement(animated, "material", name="Boat")
                ET.SubElement(animated, "look", name="Fisherboat", uv_mode="0")

                # Trajectory element
                trajectory = ET.SubElement(
                    animated, "trajectory", type="spline", playback="repeat"
                )
                vehicles[vehicle_name] = trajectory

            # Add keypoints to the trajectory
            trajectory = vehicles[vehicle_name]
            position = details["center_position_m"]
            heading = details["heading_rad"] + 3.14159  # Add 180 degrees (in radians) to the heading

            ET.SubElement(
                trajectory,
                "keypoint",
                time=str(time_key),
                xyz=f"{position[0]} {position[1]} 0.5",
                rpy=f"-1.57 0.0 {heading}"
            )

    # Generate the XML tree and write it to a file
    tree = ET.ElementTree(root)
    ET.indent(tree, space="    ", level=0)  # Pretty print the XML
    tree.write(output_file, encoding="utf-8", xml_declaration=True)
    print(f"Dynamic XML file has been generated: {output_file}")


def parse_static_to_xml(json_file, output_file):
    # Read the JSON file
    with open(json_file, 'r') as f:
        json_data = json.load(f)

    # Create the root XML element
    root = ET.Element("scenario")

    first_wall = True

    # Process JSON data and build XML structure for statics
    for wall_name, wall_data in json_data.items():
        if wall_data["type"] == "wall":
            dimensions = wall_data["sides_m"]

            if wall_name == "OuterBorder":

                # Handle outer boundary as separate walls
                for i, side in enumerate(dimensions):
                    start, end = side
                    length = ((end[0] - start[0])**2 + (end[1] - start[1])**2)**0.5
                    center_x = (start[0] + end[0]) / 2
                    center_y = (start[1] + end[1]) / 2
                    angle = 0.0 if end[1] == start[1] else 1.5708  # 90 degrees in radians for vertical walls

                    static = ET.SubElement(root, "static", name=f"{wall_name}_side{i}", type="box")
                    ET.SubElement(static, "dimensions", xyz=f"{length} 0.1 1.0")
                    ET.SubElement(static, "material", name="Steel")
                    ET.SubElement(static, "look", name="Gray")
                    ET.SubElement(static, "world_transform", xyz=f"{center_x} {center_y} 0.0", rpy=f"0.0 0.0 {angle}")
                
                print(f"Found outer border, drawing at {center_x} {center_y} with length {length}")

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

                if wall_name in model_replacement:
                    print(f"Found {wall_name} in model_replacement")
                    model_file = model_replacement[wall_name]
                    static = ET.SubElement(root, "static", name=wall_name, type="model")

                    physical = ET.SubElement(static, "physical")
                    ET.SubElement(physical, "mesh", filename=f"obstacles/{model_file}.obj", scale="1.0")
                    ET.SubElement(physical, "origin", xyz="0.0 0.0 0.0", rpy="0.0 0.0 0.0")

                    visual = ET.SubElement(static, "visual")
                    ET.SubElement(visual, "mesh", filename=f"obstacles/{model_file}.obj", scale="1.0")
                    ET.SubElement(visual, "origin", xyz="0.0 0.0 0.0", rpy="0.0 0.0 0.0")

                    # hard coded for now
                    ET.SubElement(static, "material", name="Stone")
                    ET.SubElement(static, "look", name=model_file, uv_mode="0")
                    ET.SubElement(static, "world_transform", xyz=f"{center_x} {center_y} 0.5", rpy="-1.57 0.0 0.0")
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

json_input = "scenario_parser/dynamics.json"
output_file = "metadata/dynamics.scn"
json_to_xml(json_input, output_file)

# list for model replacements 
# name should correspont to both the .obj file and the .png / .jpg (texture) file
# .obj file must be located in data/obstacles
model_replacement = {
    "Pier2": "rocks_line",
    "Building1": "rock3"
}

json_input_statics = "scenario_parser/statics.json"
output_file_statics = "metadata/statics.scn"
parse_static_to_xml(json_input_statics, output_file_statics)
