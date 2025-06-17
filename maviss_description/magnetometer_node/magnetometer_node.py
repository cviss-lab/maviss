#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32
import numpy as np
import math

class CompassNode(Node):
    def __init__(self):
        super().__init__('compass_node')
        self.heading_pub = self.create_publisher(Float32, '/magnetometer/heading', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/magnetometer/data', 10)
        self.create_subscription(Imu, '/imu/imu/out', self.imu_callback, 10)
        # Universal Earth's magnetic field in ENU (Tesla)
        self.B_enu = np.array([0.0, 25e-6, 43.3e-6])

    def imu_callback(self, msg):
        # Get quaternion from IMU
        q = msg.orientation
        quat = np.array([q.w, q.x, q.y, q.z])  # [w, x, y, z]

        # Convert quaternion to rotation matrix
        R = self.quaternion_to_rotation_matrix(quat)

        # Rotate Earth's magnetic field vector into body frame
        B_body = R.T @ self.B_enu

        # Tilt compensation: use all 3 axes
        # Compute roll and pitch from the quaternion
        roll, pitch = self.get_roll_pitch(quat)

        # Compensate the magnetometer readings
        mx, my, mz = B_body
        mx2 = mx * math.cos(pitch) + mz * math.sin(pitch)
        my2 = mx * math.sin(roll) * math.sin(pitch) + my * math.cos(roll) - mz * math.sin(roll) * math.cos(pitch)

        # Heading in radians (North=0°, East=90°)
        heading_rad = math.atan2(my2, mx2)
        heading_deg = math.degrees(heading_rad)
        if heading_deg < 0:
            heading_deg += 360.0

        # Only publish heading if there are subscribers
        if self.heading_pub.get_subscription_count() > 0:
            heading_msg = Float32()
            heading_msg.data = heading_deg
            self.heading_pub.publish(heading_msg)

        # Only publish MagneticField if there are subscribers
        if self.mag_pub.get_subscription_count() > 0:
            mag_msg = MagneticField()
            mag_msg.header = msg.header
            mag_msg.header.frame_id = 'base_link'
            mag_msg.magnetic_field.x = float(mx)
            mag_msg.magnetic_field.y = float(my)
            mag_msg.magnetic_field.z = float(mz)
            self.mag_pub.publish(mag_msg)

        # Log heading for debug
        #self.get_logger().info(f"Compass heading (deg): {heading_deg:.2f}")

    @staticmethod
    def quaternion_to_rotation_matrix(q):
        w, x, y, z = q
        return np.array([
            [1 - 2*(y**2 + z**2),     2*(x*y - z*w),         2*(x*z + y*w)],
            [    2*(x*y + z*w),   1 - 2*(x**2 + z**2),       2*(y*z - x*w)],
            [    2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x**2 + y**2)]
        ])

    @staticmethod
    def get_roll_pitch(q):
        # Returns roll and pitch in radians from quaternion [w, x, y, z]
        w, x, y, z = q
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
        return roll, pitch

def main(args=None):
    rclpy.init(args=args)
    node = CompassNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

