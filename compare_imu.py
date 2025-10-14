#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import GimbalDeviceAttitudeStatus, VehicleAttitude, SensorCombined
from sensor_msgs.msg import Imu
import math
import sys

def quat_to_euler(w, x, y, z):
    """Convert quaternion to Euler angles (roll, pitch, yaw) in FRD/NED convention"""
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

    return roll, pitch, yaw

class IMUComparer(Node):
    def __init__(self):
        super().__init__('imu_comparer')

        # QoS profile for PX4 topics (best_effort)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.vehicle_quat = None
        self.gimbal_quat = None
        self.vehicle_accel = None
        self.gimbal_accel = None
        self.received_both = False

        self.vehicle_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.vehicle_callback,
            qos_profile)

        self.gimbal_sub = self.create_subscription(
            GimbalDeviceAttitudeStatus,
            '/fmu/out/gimbal_device_attitude_status',
            self.gimbal_callback,
            qos_profile)

        # Subscribe to vehicle IMU for accelerometer data
        self.vehicle_imu_sub = self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.vehicle_imu_callback,
            qos_profile)

        # Note: Gimbal accelerometer data is not currently published to ROS2.
        # GZGimbal.cpp subscribes to the gimbal IMU but only extracts orientation and angular velocity.
        # To add gimbal accel, would need to modify GZGimbal.cpp to extract linear_acceleration
        # and add it to GimbalDeviceAttitudeStatus message definition.

    def vehicle_callback(self, msg):
        if not self.received_both:
            self.vehicle_quat = (msg.q[0], msg.q[1], msg.q[2], msg.q[3])
            self.check_and_display()

    def gimbal_callback(self, msg):
        if not self.received_both:
            self.gimbal_quat = (msg.q[0], msg.q[1], msg.q[2], msg.q[3])
            self.check_and_display()

    def vehicle_imu_callback(self, msg):
        if not self.received_both:
            # SensorCombined contains accelerometer data in m/s^2
            self.vehicle_accel = (msg.accelerometer_m_s2[0], msg.accelerometer_m_s2[1], msg.accelerometer_m_s2[2])

    def check_and_display(self):
        if self.vehicle_quat is not None and self.gimbal_quat is not None and not self.received_both:
            self.received_both = True

            # Vehicle orientation
            v_roll, v_pitch, v_yaw = quat_to_euler(*self.vehicle_quat)

            # Gimbal orientation
            g_roll, g_pitch, g_yaw = quat_to_euler(*self.gimbal_quat)

            # Calculate difference
            d_roll = g_roll - v_roll
            d_pitch = g_pitch - v_pitch
            d_yaw = g_yaw - v_yaw

            # Normalize yaw difference to [-pi, pi]
            while d_yaw > math.pi:
                d_yaw -= 2 * math.pi
            while d_yaw < -math.pi:
                d_yaw += 2 * math.pi

            print("=" * 80)
            print("VEHICLE ORIENTATION:")
            print(f"  Roll:  {math.degrees(v_roll):7.2f}° ({v_roll:8.5f} rad)")
            print(f"  Pitch: {math.degrees(v_pitch):7.2f}° ({v_pitch:8.5f} rad)")
            print(f"  Yaw:   {math.degrees(v_yaw):7.2f}° ({v_yaw:8.5f} rad)")
            print(f"  Quaternion: [w={self.vehicle_quat[0]:.6f}, x={self.vehicle_quat[1]:.6e}, " +
                  f"y={self.vehicle_quat[2]:.6e}, z={self.vehicle_quat[3]:.6e}]")
            if self.vehicle_accel is not None:
                print(f"  Accel: [x={self.vehicle_accel[0]:7.3f}, y={self.vehicle_accel[1]:7.3f}, z={self.vehicle_accel[2]:7.3f}] m/s²")

            print("\nGIMBAL ORIENTATION:")
            print(f"  Roll:  {math.degrees(g_roll):7.2f}° ({g_roll:8.5f} rad)")
            print(f"  Pitch: {math.degrees(g_pitch):7.2f}° ({g_pitch:8.5f} rad)")
            print(f"  Yaw:   {math.degrees(g_yaw):7.2f}° ({g_yaw:8.5f} rad)")
            print(f"  Quaternion: [w={self.gimbal_quat[0]:.6f}, x={self.gimbal_quat[1]:.6e}, " +
                  f"y={self.gimbal_quat[2]:.6e}, z={self.gimbal_quat[3]:.6e}]")
            if self.gimbal_accel is not None:
                print(f"  Accel: [x={self.gimbal_accel[0]:7.3f}, y={self.gimbal_accel[1]:7.3f}, z={self.gimbal_accel[2]:7.3f}] m/s²")

            print("\nDIFFERENCE (Gimbal - Vehicle):")
            print(f"  ΔRoll:  {math.degrees(d_roll):7.2f}° ({d_roll:8.5f} rad)")
            print(f"  ΔPitch: {math.degrees(d_pitch):7.2f}° ({d_pitch:8.5f} rad)")
            print(f"  ΔYaw:   {math.degrees(d_yaw):7.2f}° ({d_yaw:8.5f} rad)")
            print("=" * 80)

            sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = IMUComparer()

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
