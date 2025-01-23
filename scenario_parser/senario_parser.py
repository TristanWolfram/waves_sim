import json
import os
import xml.dom.minidom as minidom
import xml.etree.ElementTree as ET


def convert_json_to_stonefish_xml():

    print(os.getcwd())

    # data = json.loads(json_data)

    root = ET.Element("senario")

    animated = ET.SubElement(root, "animated")
    physical = ET.SubElement(animated, "physical")
    ET.SubElement(
        physical, "mesh", {"filename": "boats/fisher_boat_phys.obj", "scale": "5.0"}
    )
    ET.SubElement(physical, "origin", {"xyz": "0.0 0.0 0.0", "rpy": "0.0 0.0 0.0"})

    visual = ET.SubElement(animated, "visual")
    ET.SubElement(visual, "mesh", {"filename": "boats/fisher_boat.obj", "scale": "5.0"})
    ET.SubElement(visual, "origin", {"xyz": "0.0 0.0 0.0", "rpy": "0.0 0.0 0.0"})

    ET.SubElement(root, "material", {"name": "Boat"})
    ET.SubElement(root, "look", {"name": "Fisherboat", "uv_mode": "0"})

    trajectory = ET.SubElement(
        animated, "trajectory", {"type": "spline", "playback": "repeat"}
    )
    ET.SubElement(
        trajectory,
        "keypoint",
        {"time": "1.0", "xyz": "0.0 0.0 0.5", "rpy": "-1.57 0.0 0.0"},
    )

    xml_str = ET.tostring(root, encoding="unicode")
    pretty_xml = minidom.parseString(xml_str).toprettyxml(indent="  ")

    # Write to file
    with open("scenario_parser/output/dynamics.scn", "w") as f:
        f.write(pretty_xml)

    return pretty_xml


output_xml = convert_json_to_stonefish_xml()
print(output_xml)
