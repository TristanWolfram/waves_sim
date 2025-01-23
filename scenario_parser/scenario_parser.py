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
                    physical, "mesh", filename="boats/fisher_boat_phys.obj", scale="5.0"
                )
                ET.SubElement(physical, "origin", xyz="0.0 0.0 0.0", rpy="0.0 0.0 0.0")

                # Visual element
                visual = ET.SubElement(animated, "visual")
                ET.SubElement(
                    visual, "mesh", filename="boats/fisher_boat.obj", scale="5.0"
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
            heading = details["heading_rad"]

            ET.SubElement(
                trajectory,
                "keypoint",
                time=str(time_key),
                xyz=f"{position[0]} {position[1]} 0.5",
                rpy=f"0.0 0.0 {heading}",
            )

    # Generate the XML tree and write it to a file
    tree = ET.ElementTree(root)
    ET.indent(tree, space="    ", level=0)  # Pretty print the XML
    tree.write(output_file, encoding="utf-8", xml_declaration=True)


json_input = "scenario_parser/input/dynamics.json"

output_file = "scenario_parser/output/dynamics.scn"
json_to_xml(json_input, output_file)
