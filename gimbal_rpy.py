#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import GimbalDeviceAttitudeStatus
import math
import sys

class GimbalRPYReader(Node):
    def __init__(self):
        super().__init__('gimbal_rpy_reader')

        # QoS profile matching PX4 topics (best_effort)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            GimbalDeviceAttitudeStatus,
            '/fmu/out/gimbal_device_attitude_status',
            self.listener_callback,
            qos_profile)
        self.received = False

    def listener_callback(self, msg):
        if self.received:
            return
        self.received = True

        # Quaternion from message (w, x, y, z)
        w, x, y, z = msg.q[0], msg.q[1], msg.q[2], msg.q[3]

        # Convert to Euler angles (roll, pitch, yaw) in FRD/NED convention
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        print(f"Gimbal Orientation:")
        print(f"  Roll:  {math.degrees(roll):7.2f}° ({roll:8.5f} rad)")
        print(f"  Pitch: {math.degrees(pitch):7.2f}° ({pitch:8.5f} rad)")
        print(f"  Yaw:   {math.degrees(yaw):7.2f}° ({yaw:8.5f} rad)")
        print(f"\nQuaternion: [w={w:.6f}, x={x:.6e}, y={y:.6e}, z={z:.6e}]")

        # Exit immediately
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = GimbalRPYReader()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
