# da_vinci_tool_integration (KUKA Med7)

This ROS 2 package provides a combined robot description and launch setup for the KUKA LBR Med7 arm with a da Vinci PSM tool and adaptor, fully self-contained for easy simulation and visualization in RViz2.

---

## Features

- Combines LBR Med7, da Vinci PSM, and adaptor into a single robot model
- All adaptor URDF and mesh files are included locally (no external adaptor package needed)
- Example launch file for visualization in RViz2

---

## Dependencies

You must have the following packages in your ROS 2 workspace:

- **[lbr_stack](https://github.com/lbr-stack/lbr_fri_ros2_stack)**
  - Provides the LBR Med7 robot description and related resources.
- **[dvrk_urdf](https://github.com/shashank3199/dvrk_urdf)**
  - Provides the da Vinci PSM, ECM, MTM, SUJ, and patient cart URDFs and related resources.
- **Standard ROS 2 packages:**
  - `robot_state_publisher`
  - `joint_state_publisher_gui`
  - `rviz2`

---

## Workspace Setup Example

```bash
# Clone dependencies into your workspace
cd ~/my_ros2_ws/src

# LBR stack (required)
git clone https://github.com/lbr-stack/lbr_fri_ros2_stack.git

# dVRK URDF (required)
git clone https://github.com/shashank3199/dvrk_urdf.git

# This package
git clone https://github.com/suphasitpp/da_vinci_tool_integration2.git

# Build the workspace
cd ~/my_ros2_ws
colcon build --symlink-install

# Source the workspace
source install/local_setup.bash
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
- If you add or update dependencies, rebuild your workspace and re-source your setup file.

---

## License

[MIT License](LICENSE) (or your chosen license)

---

## Contact

For questions or contributions, open an issue or pull request on [GitHub](https://github.com/suphasitpp/da_vinci_tool_integration2). 