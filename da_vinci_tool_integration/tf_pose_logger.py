#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
import time
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class TFPoseLogger(Node):
    def __init__(self):
        super().__init__('tf_pose_logger')

        # Initialize TF buffer and listener with error handling
        try:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.get_logger().info("✅ TF buffer and listener initialized successfully")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize TF components: {e}")
            raise

        # Create subscription with error handling
        try:
            self.subscription = self.create_subscription(
                PoseStamped,
                '/tool_target',
                self.pose_callback,
                10
            )
            self.get_logger().info("✅ Subscription to /tool_target created successfully")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to create subscription: {e}")
            raise

        # Initialize counters for monitoring
        self.pose_count = 0
        self.success_count = 0
        self.error_count = 0

        self.get_logger().info("🔍 TFPoseLogger started. Waiting for /tool_target pose...")

    def pose_callback(self, msg):
        """Handle incoming pose messages with comprehensive error checking"""
        self.pose_count += 1
        self.get_logger().info(f"📤 Received target pose #{self.pose_count}. Waiting for TF update...")
        
        # Validate incoming message
        if not self._validate_pose_message(msg):
            self.error_count += 1
            return

        # Wait for TF update with timeout
        if not self._wait_for_tf_update():
            self.error_count += 1
            return

        # Perform TF lookup with comprehensive error handling
        transform = self._lookup_transform_safe()
        if transform is None:
            self.error_count += 1
            return

        # Log the successful transform
        self._log_transform_details(transform)
        self.success_count += 1

    def _validate_pose_message(self, msg):
        """Validate the incoming pose message"""
        try:
            # Check if message is not None
            if msg is None:
                self.get_logger().error("❌ Received None pose message")
                return False

            # Check header
            if not hasattr(msg, 'header') or msg.header is None:
                self.get_logger().error("❌ Pose message missing header")
                return False

            # Check frame_id
            if not hasattr(msg.header, 'frame_id') or not msg.header.frame_id:
                self.get_logger().error("❌ Pose message missing frame_id")
                return False

            # Check pose
            if not hasattr(msg, 'pose') or msg.pose is None:
                self.get_logger().error("❌ Pose message missing pose data")
                return False

            # Check position
            if not hasattr(msg.pose, 'position') or msg.pose.position is None:
                self.get_logger().error("❌ Pose message missing position data")
                return False

            # Check orientation
            if not hasattr(msg.pose, 'orientation') or msg.pose.orientation is None:
                self.get_logger().error("❌ Pose message missing orientation data")
                return False

            # Validate quaternion (basic check)
            q = msg.pose.orientation
            quat_norm = q.x**2 + q.y**2 + q.z**2 + q.w**2
            if abs(quat_norm - 1.0) > 0.1:  # Allow some tolerance
                self.get_logger().warning(f"⚠️ Quaternion may not be normalized (norm={quat_norm:.6f})")

            self.get_logger().info(f"✅ Pose message validated successfully")
            return True

        except Exception as e:
            self.get_logger().error(f"❌ Error validating pose message: {e}")
            return False

    def _wait_for_tf_update(self):
        """Wait for TF update with timeout and error handling"""
        try:
            # Wait a short time for TF to update
            time.sleep(0.2)
            
            # Check if TF buffer is available
            if not self.tf_buffer.can_transform('lbr_link_0', 'PSM_tool_virtual_tip', rclpy.time.Time()):
                self.get_logger().warning("⚠️ TF transform not yet available, waiting...")
                time.sleep(0.1)  # Additional wait
                
                if not self.tf_buffer.can_transform('lbr_link_0', 'PSM_tool_virtual_tip', rclpy.time.Time()):
                    self.get_logger().error("❌ TF transform still not available after waiting")
                    return False

            self.get_logger().info("✅ TF transform is available")
            return True

        except Exception as e:
            self.get_logger().error(f"❌ Error waiting for TF update: {e}")
            return False

    def _lookup_transform_safe(self):
        """Perform TF lookup with comprehensive error handling"""
        try:
            # Try to get the transform
            trans = self.tf_buffer.lookup_transform(
                'lbr_link_0',
                'PSM_tool_virtual_tip',
                rclpy.time.Time()
            )
            return trans

        except LookupException as e:
            self.get_logger().error(f"❌ TF LookupException: {e}")
            self.get_logger().error("   This usually means the transform doesn't exist")
            return None

        except ConnectivityException as e:
            self.get_logger().error(f"❌ TF ConnectivityException: {e}")
            self.get_logger().error("   This usually means the TF tree is not connected")
            return None

        except ExtrapolationException as e:
            self.get_logger().error(f"❌ TF ExtrapolationException: {e}")
            self.get_logger().error("   This usually means the transform is too old")
            return None

        except Exception as e:
            self.get_logger().error(f"❌ Unexpected TF error: {e}")
            return None

    def _log_transform_details(self, trans):
        """Log the transform details with full precision"""
        try:
            # Extract translation and rotation
            t = trans.transform.translation
            r = trans.transform.rotation

            # Validate transform data
            if t is None or r is None:
                self.get_logger().error("❌ Transform contains None translation or rotation")
                return

            # Check for NaN or infinite values
            if (not self._is_finite(t.x) or not self._is_finite(t.y) or not self._is_finite(t.z) or
                not self._is_finite(r.x) or not self._is_finite(r.y) or not self._is_finite(r.z) or not self._is_finite(r.w)):
                self.get_logger().error("❌ Transform contains NaN or infinite values")
                return

            # Log the full transform with statistics
            self.get_logger().info("\n🤖 Full Actual Pose from TF:")
            self.get_logger().info(f"  Translation:")
            self.get_logger().info(f"    x: {t.x:.16f}")
            self.get_logger().info(f"    y: {t.y:.16f}")
            self.get_logger().info(f"    z: {t.z:.16f}")
            self.get_logger().info(f"  Rotation (quaternion):")
            self.get_logger().info(f"    x: {r.x:.16f}")
            self.get_logger().info(f"    y: {r.y:.16f}")
            self.get_logger().info(f"    z: {r.z:.16f}")
            self.get_logger().info(f"    w: {r.w:.16f}")

            # Log statistics
            self.get_logger().info(f"\n📊 Statistics:")
            self.get_logger().info(f"  Total poses received: {self.pose_count}")
            self.get_logger().info(f"  Successful transforms: {self.success_count}")
            self.get_logger().info(f"  Failed transforms: {self.error_count}")
            if self.pose_count > 0:
                success_rate = (self.success_count / self.pose_count) * 100
                self.get_logger().info(f"  Success rate: {success_rate:.1f}%")

        except Exception as e:
            self.get_logger().error(f"❌ Error logging transform details: {e}")

    def _is_finite(self, value):
        """Check if a value is finite (not NaN or infinite)"""
        import math
        return not (math.isnan(value) or math.isinf(value))

    def __del__(self):
        """Cleanup method"""
        try:
            self.get_logger().info("🔄 TFPoseLogger shutting down...")
        except:
            pass  # Logger might not be available during shutdown


def main(args=None):
    """Main function with error handling"""
    try:
        rclpy.init(args=args)
        node = TFPoseLogger()
        
        # Log startup information
        node.get_logger().info("🚀 Starting TFPoseLogger node...")
        node.get_logger().info("📋 Monitoring:")
        node.get_logger().info("  - /tool_target topic for pose messages")
        node.get_logger().info("  - TF transform from lbr_link_0 to PSM_tool_virtual_tip")
        node.get_logger().info("  - Full precision logging (16 decimal places)")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    except Exception as e:
        print(f"❌ Fatal error: {e}")
    finally:
        try:
            rclpy.shutdown()
            print("✅ TFPoseLogger shutdown complete")
        except:
            pass


if __name__ == '__main__':
    main() 