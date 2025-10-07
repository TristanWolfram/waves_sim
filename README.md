# WAVES - Waterborn Agents in Virtual Environment Simulations

## Simulation Package

This repository contains the `rbi_sim` (rbi - realistic boat interaction) ROS 2 package which provides the Stonfish configutration together with support nodes for the virtual sensors. It contains launch files, metadata, sensor conversion utilities, and helper scripts for generating `.scn` scenario descriptions that can be consumed by Stonefish.

## Package layout

```
.
├── data/                 # Static meshes and textures referenced by scenarios
├── launch/               # ROS 2 launch descriptions for starting the simulator stack
├── metadata/             # Generated `.scn` scenario files for Stonefish
├── scenario_parser/      # Python utilities for turning JSON scenarios into Stonefish metadata
├── scenarios/            # Hand-authored scenario templates bundled with the package
├── src/                  # C++ ROS 2 nodes (LiDAR converter and thruster allocator)
└── package.xml, CMakeLists.txt
```

## Build and runtime dependencies

The package targets ROS 2 (tested with Humble) and relies on the standard colcon workflow. The core C++ nodes depend on:

- `stonefish_ros2`
- `rclcpp`
- `sensor_msgs`
- `cv_bridge`
- `image_transport`
- `message_filters`
- `pcl_ros`/`pcl_conversions` and PCL
- `OpenCV`

Runtime launch files additionally assume the `joy` and `tf2_ros` packages are installed so that the joystick driver and static transform publisher are available. All of these dependencies are declared in `package.xml` and `CMakeLists.txt` and will be resolved automatically by ROS 2 when building inside a workspace. 

The Python-based scenario parser only requires the standard library plus `numpy`.

### Preparing a workspace

```bash
# Create a colcon workspace if you do not already have one
mkdir -p ~/rbi_ws/src
cd ~/rbi_ws/src

# Clone this repository into the workspace
git clone https://github.com/your-org/waves_sim.git

# Fetch any other dependencies (e.g. stonefish_ros2) into the workspace
# ...
```

### Building the package

```bash
cd ~/rbi_ws
source /opt/ros/<ros2-distro>/setup.bash
colcon build --packages-select rbi_sim
source install/setup.bash
```

Replace `<ros2-distro>` with the ROS 2 distribution you are using (e.g. `humble`).

## Running the simulator

The `sim.launch.py` entry point wraps the Stonefish simulator launch file and injects the local sensor-processing nodes that live in this package. Launch it with:

```bash
ros2 launch rbi_sim sim.launch.py
```

Key launch arguments:

| Argument | Default | Description |
| --- | --- | --- |
| `scn` | `ocean` | Scenario stem to load from `scenarios/<name>.scn`. Use `testing` or `plane` to switch scenes. (Use `testing` in the beginning to confirm that you isntallation works) |
| `window_res_x` / `window_res_y` | `1820` / `1000` | Render window resolution forwarded to Stonefish. |
| `rendering_quality` | `high` | Rendering preset forwarded to Stonefish. |

The launch file embeds three companion nodes:

- **`LiDAR_converter`** – Subscribes to the four simulated depth cameras (`/sim_camF_depth_*`, `/sim_camB_depth_*`, `/sim_camL_depth_*`, `/sim_camR_depth_*`) and synthesises a fused 32-beam, 360° point cloud on `/sim_LiDAR_depth/points`.
- **`thrust_allocator`** – Consumes joystick commands on `/joy` and publishes differential thrust commands on `/blueboat/thruster_forces`. Axis indices and saturation limits are exposed as ROS 2 parameters.
- **Static transform publisher** – Provides the fixed transform between the ZED camera frame and the simulated LiDAR frame for downstream consumers. 

A joystick driver (`ros2 run joy joy_node`) will be started automatically by the launch file, but you can also run it manually if needed.

## Scenario authoring

Stonefish expects `.scn` files describing dynamic actors and static obstacles. The `scenario_parser/` directory contains helpers for generating those files from structured JSON.

### Dynamic scenario generation

1. Edit the JSON template under `scenario_parser/json_files/` (e.g. `cliff-dynamics.json`) to describe the vessel trajectories, sensor load-outs, and wave settings you want. Such JSON files are generated with the help of another Master Thesis by Henrik Fjellheim. HIs Code can be found [here](https://github.com/henrfj/master_thesis_2023). I [forked](https://github.com/TristanWolfram/2d_simulation_code) of the original repository to generate boat like trajectories and adapt the sourroundings. 
2. Adjust sensor catalogues, vehicle models, or wave-noise toggles inside `scenario_parser/scenario_parser.py` if you need different defaults. 
3. Run the parser:

   ```bash
   cd scenario_parser
   python3 scenario_parser.py
   ```

   The script writes the generated dynamic scenario to `metadata/<preamble>-dynamics.scn` and logs which sensors are being attached to each vessel. 

### Static asset parsing

Static obstacle definitions can be changed by editing the `MODEL_REPLACEMENTS` table toward the bottom of `scenario_parser/scenario_parser.py`. When you re-run the parser, updated meshes and transforms will be reflected in the `.scn` output generated under `metadata/`. 

### Wave-noise model

`parsing_methods.py` implements a lightweight wave-noise synthesiser that can be enabled per scenario. Tuning parameters such as amplitudes, damping, and frequency live at the top of that file and influence the noise applied to moving vessels when `INCLUDE_WAVE_NOISE` is `True`. 

## Helpful ROS 2 commands

- Inspect available launch arguments:

  ```bash
  ros2 launch rbi_sim sim.launch.py --show-args
  ```

- Run individual support nodes for debugging:

  ```bash
  ros2 run rbi_sim LiDAR_converter
  ros2 run rbi_sim thrust_allocator
  ```

- Visualise the fused point cloud in RViz (although I would recommend using [Foxglove](https://foxglove.dev/)):

  ```bash
  ros2 run rviz2 rviz2
  ```

## License

This package is released under the MIT License, as declared in `package.xml`.
