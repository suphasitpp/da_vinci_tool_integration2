#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time

class FKQuery(Node):
    def __init__(self):
        super().__init__('fk_query')

        self.fk_client = self.create_client(GetPositionFK, '/get_position_fk')
        self.get_logger().info("Waiting for FK service...")
        self.fk_client.wait_for_service()
        self.get_logger().info("Connected to /get_position_fk")

        # Create and send request
        req = GetPositionFK.Request()
        req.fk_link_names = ['PSM_tool_virtual_tip']
        req.robot_state.joint_state.name = [
            "lbr_A1", "lbr_A2", "lbr_A3", "lbr_A4", "lbr_A5", "lbr_A6", "lbr_A7"
        ]
        req.robot_state.joint_state.position = [0.0] * 7
        req.robot_state.joint_state.header.stamp = Time(sec=0, nanosec=0)
        req.header.frame_id = 'lbr_link_0'

        future = self.fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().pose_stamped:
            pose = future.result().pose_stamped[0]
            self.get_logger().info("FK pose at [0,0,0,0,0,0,0]:")
            print("\n--- COPY THIS INTO /tool_target ---")
            print(f"""header:
  frame_id: '{pose.header.frame_id}'
pose:
  position:
    x: {pose.pose.position.x}
    y: {pose.pose.position.y}
    z: {pose.pose.position.z}
  orientation:
    x: {pose.pose.orientation.x}
    y: {pose.pose.orientation.y}
    z: {pose.pose.orientation.z}
    w: {pose.pose.orientation.w}
""")
        else:
            self.get_logger().error("FK call failed or returned no result.")

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FKQuery()


if __name__ == '__main__':
    main() 