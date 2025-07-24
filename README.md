# da_vinci_tool_integration

This ROS 2 package provides a combined robot description and launch setup for the KUKA LBR Med7 arm with a da Vinci PSM tool and adaptor. It works in conjunction with the `moveit_config` package to provide complete motion planning and kinematic solving capabilities for surgical robotics applications.

---

## Features

- Combines LBR Med7, da Vinci PSM, and adaptor into a single robot model
- All adaptor URDF and mesh files are included locally (no external adaptor package needed)
- Example launch file for visualization in RViz2
- Forward and inverse kinematics solvers for motion planning
- Interactive target marker for intuitive robot control
- Pre-configured RViz settings for surgical robotics visualization

---

## Dependencies

You must have the following packages in your ROS 2 workspace:

- **[LBR-Stack](https://github.com/lbr-stack)** - KUKA LBR robot descriptions and control
- **[dvrk_urdf](https://github.com/shashank3199/dvrk_urdf)** - da Vinci robot descriptions
- **[moveit_config](https://github.com/suphasitpp/moveit_config)** - MoveIt configuration for motion planning
- **MoveIt 2** - For motion planning and kinematic solving capabilities
- **Standard ROS 2 packages:** robot_state_publisher, joint_state_publisher_gui, rviz2, interactive_markers

---

## Workspace Setup Example

```bash
# Create workspace
mkdir -p ~/my_ros2_ws/src && cd ~/my_ros2_ws

# Set up LBR stack with proper dependencies (includes fri and lbr_fri_idl)
source /opt/ros/humble/setup.bash
export FRI_CLIENT_VERSION=1.15
vcs import src --input https://raw.githubusercontent.com/lbr-stack/lbr_fri_ros2_stack/humble/lbr_fri_ros2_stack/repos-fri-${FRI_CLIENT_VERSION}.yaml

# Clone additional dependencies
cd src
git clone https://github.com/shashank3199/dvrk_urdf.git
git clone https://github.com/suphasitpp/da_vinci_tool_integration.git
git clone https://github.com/suphasitpp/moveit_config.git

# Build the workspace
cd ..
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

---

## Usage Examples

### Basic Visualization
```bash
# Launch RViz with combined robot model
ros2 launch da_vinci_tool_integration med7_combined_simple.launch.py
```

### Motion Planning with MoveIt
```bash
# Launch complete MoveIt demo with motion planning
ros2 launch moveit_config demo.launch.py

# Or launch MoveIt move_group server separately
ros2 launch moveit_config move_group.launch.py
```

### Kinematic Solvers
```bash
# Run forward kinematics solver
ros2 run da_vinci_tool_integration fk_query.py

# Run inverse kinematics solver  
ros2 run da_vinci_tool_integration ik_solver.py

# Launch interactive target marker
ros2 run da_vinci_tool_integration interactive_target_marker.py
```

### Integrated Workflow
For complete motion planning and control:
1. **Launch MoveIt:** `ros2 launch moveit_config demo.launch.py`
2. **Use kinematic tools:** Run the IK/FK solvers for path validation
3. **Interactive control:** Use the target marker for intuitive positioning

The packages work together to provide complete surgical robot simulation and motion planning capabilities.

---

## Package Architecture

This package works as part of a two-package system:

### This Package (`da_vinci_tool_integration`)
- **Robot Description:** URDF files for LBR Med7 + da Vinci PSM + custom adaptor
- **Kinematic Tools:** Forward/inverse kinematics solvers and interactive markers
- **Visualization:** Launch files and RViz configurations

### Companion Package (`moveit_config`)
- **Motion Planning:** MoveIt configuration for path planning and execution
- **Joint Limits:** Velocity and acceleration constraints
- **Planning Algorithms:** Optimized for surgical robotics applications

### Python Modules (This Package)
- **`fk_query.py`** - Forward kinematics solver for end-effector pose calculation
- **`ik_solver.py`** - Inverse kinematics solver for joint position calculation  
- **`interactive_target_marker.py`** - Interactive 6DOF marker for target positioning

### URDF & Configuration Files (This Package)
- **`urdf/`** - Robot descriptions including LBR Med7, PSM tool, and custom adaptor
- **`launch/`** - Launch files for robot visualization and control
- **`rviz/`** - Pre-configured RViz settings optimized for surgical robotics

---

## Notes

- **Package Integration:** This package provides robot description and kinematic tools, while the companion `moveit_config` package provides motion planning configuration.
- The adaptor URDF and mesh files are included in `urdf/adaptor/` within this package.
- You do **not** need to clone or build any separate adaptor package.
- The `fri` and `lbr_fri_idl` packages are automatically included when using the official LBR stack setup method.
- **Motion Planning:** For full motion planning capabilities, use both packages together - launch MoveIt from `moveit_config` and use the kinematic tools from this package.
- If you encounter build errors related to missing FRI client, ensure you're using the proper setup method above.

---

## License

[MIT License](LICENSE) (or your chosen license)

---

## Contact

For questions or contributions, open an issue or pull request on [GitHub](https://github.com/suphasitpp/da_vinci_tool_integration). 
