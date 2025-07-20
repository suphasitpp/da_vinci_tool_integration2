# da_vinci_tool_integration

This ROS 2 package provides a combined robot description and launch setup for the KUKA LBR Med7 arm with a da Vinci PSM tool and adaptor, fully self-contained for easy simulation and visualization in RViz2.

---

## Features

- Combines LBR Med7, da Vinci PSM, and adaptor into a single robot model
- All adaptor URDF and mesh files are included locally (no external adaptor package needed)
- Example launch file for visualization in RViz2

---

## Dependencies

You must have the following packages in your ROS 2 workspace:

- **[LBR-Stack](https://github.com/lbr-stack)** - KUKA LBR robot descriptions and control
- **[dvrk_urdf](https://github.com/shashank3199/dvrk_urdf)** - da Vinci robot descriptions
- **Standard ROS 2 packages:** robot_state_publisher, joint_state_publisher_gui, rviz2

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
git clone https://github.com/suphasitpp/da_vinci_tool_integration2.git

# Build the workspace
cd ..
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

---

## Launch Example

```bash
ros2 launch da_vinci_tool_integration med7_combined_simple.launch.py
```

This will open RViz2 with the combined robot model.

---

## Notes

- The adaptor URDF and mesh files are included in `urdf/adaptor/` within this package.
- You do **not** need to clone or build any separate adaptor package.
- The `fri` and `lbr_fri_idl` packages are automatically included when using the official LBR stack setup method.
- If you encounter build errors related to missing FRI client, ensure you're using the proper setup method above.

---

## License

[MIT License](LICENSE) (or your chosen license)

---

## Contact

For questions or contributions, open an issue or pull request on [GitHub](https://github.com/suphasitpp/da_vinci_tool_integration2). 