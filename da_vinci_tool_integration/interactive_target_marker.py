#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

class ToolTargetMarker(Node):
    def __init__(self):
        super().__init__('tool_target_marker')
        self.server = InteractiveMarkerServer(self, "tool_target_marker")
        self.pose_pub = self.create_publisher(PoseStamped, "/tool_target", 10)

        self.marker = InteractiveMarker()
        self.marker.header.frame_id = "lbr_link_0"
        self.marker.name = "tool_target"
        self.marker.description = "Tool Target Pose"
        self.marker.scale = 0.2
        self.marker.pose.position.x = 0.2
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.2
        self.marker.pose.orientation.w = 1.0

        # ✅ Add central visual sphere with 3D drag capability
        visual_marker = Marker()
        visual_marker.type = Marker.SPHERE
        visual_marker.scale.x = 0.05
        visual_marker.scale.y = 0.05
        visual_marker.scale.z = 0.05
        visual_marker.color.r = 1.0
        visual_marker.color.g = 0.2
        visual_marker.color.b = 0.2
        visual_marker.color.a = 0.8

        # Combine visual and drag functionality in one control
        drag_control = InteractiveMarkerControl()
        drag_control.name = "move_3d"
        drag_control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        drag_control.always_visible = True
        drag_control.markers.append(visual_marker)
        self.marker.controls.append(drag_control)

        # ✅ Axis-specific rotation + movement
        for axis in ['x', 'y', 'z']:
            move = InteractiveMarkerControl()
            move.name = f"move_{axis}"
            move.orientation.w = 1.0
            move.orientation.x = 1.0 if axis == 'x' else 0.0
            move.orientation.y = 1.0 if axis == 'y' else 0.0
            move.orientation.z = 1.0 if axis == 'z' else 0.0
            move.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            self.marker.controls.append(move)

            rotate = InteractiveMarkerControl()
            rotate.name = f"rotate_{axis}"
            rotate.orientation.w = 1.0
            rotate.orientation.x = 1.0 if axis == 'x' else 0.0
            rotate.orientation.y = 1.0 if axis == 'y' else 0.0
            rotate.orientation.z = 1.0 if axis == 'z' else 0.0
            rotate.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            self.marker.controls.append(rotate)

        self.server.insert(self.marker)
        self.server.setCallback(self.marker.name, self.feedback_callback)
        self.server.applyChanges()
        self.get_logger().info("Interactive marker ready. Drag the marker in RViz to control tool target.")

    def feedback_callback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            pose = PoseStamped()
            pose.header.frame_id = "lbr_link_0"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose = feedback.pose
            self.pose_pub.publish(pose)
            self.get_logger().info(f"Published new tool target pose: x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}, z={pose.pose.position.z:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = ToolTargetMarker()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main() 