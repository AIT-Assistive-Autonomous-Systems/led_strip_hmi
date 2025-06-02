# led\_strip\_hmi

## Overview

The **led\_strip\_hmi** repository is a collection of ROS 2 packages enabling the configuration, projection, and visualization of virtual LED strips for robotics applications. By defining arbitrary strip geometries and projecting 3D detections (e.g., human poses, LaserScans) onto these strips, you can enhance situational awareness and safety around your robot.

This repo contains four core packages:

| Package                            | Purpose                                                                                                            | README Link                                                     |
| ---------------------------------- | ------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------- |
| **led\_strip\_hmi\_common**        | Core utilities for loading and validating strip configurations, coordinate transforms, and virtual strip geometry. | [Common README](./led_strip_hmi_common/README.md)               |
| **led\_strip\_hmi\_msgs**          | Custom ROS 2 message definitions for strip configurations, projection results, and visual effects.                 | [Msgs README](./led_strip_hmi_msgs/README.md)                   |
| **led\_strip\_hmi\_projector**     | Node and libraries to project 3D detections onto virtual strips, publishing LED indices and debug images.          | [Projector README](./led_strip_hmi_projector/README.md)         |
| **led\_strip\_hmi\_visualization** | Node to render LED strips in RViz, highlighting active LEDs based on projection data.                              | [Visualization README](./led_strip_hmi_visualization/README.md) |

## Repository Structure

```
led_strip_hmi/             # root of this git repo
├── LICENSE              
├── README.md             
├── led_strip_hmi_common
├── led_strip_hmi_msgs
├── led_strip_hmi_projector
└── led_strip_hmi_visualization
```

Each subfolder under `src/led_strip_hmi` is a standalone ROS 2 package with its own `package.xml`, `setup.py` (or `CMakeLists.txt`), tests, and README.

## Getting Started

1. **Clone the repository**

   ```bash
   git clone <repo_url>
   cd led_strip_hmi
   ```
2. **Install dependencies**

   ```bash
   rosdep update && \
     rosdep install --from-paths src --ignore-src -r -y
   ```
3. **Build**

   ```bash
   colcon build
   source install/setup.bash
   ```
4. **Run a Demo**
   Use one of the demo launch files in the **led\_strip\_hmi\_demos** package (e.g., `ros2 launch led_strip_hmi_demos caripu.launch.py`).

## License

See [LICENSE](LICENSE) for details.
