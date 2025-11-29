#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
import numpy as np
import math

class QuadrupedIKMapped(Node):
    def __init__(self):
        super().__init__('quadruped_ik_mapped')
        
        # --- ROBOT GEOMETRY (Meters) ---
        self.upper_len = 0.13002 
        self.lower_len = 0.16078 
        
        # --- CALIBRATION ---
        self.thigh_offset = 0.0 
        
        # --- JOINT INDICES ---
        self.joint_indices = {
            'bl': [0, 4, 8],   'br': [1, 5, 9],
            'fl': [2, 6, 10],  'fr': [3, 7, 11]
        }

        # --- 1. HIP CONFIGURATION ---
        self.hip_offsets = {
            'fl': {'x': 0.1, 'y': 0.06}, 'fr': {'x': 0.1, 'y': -0.06},
            'bl': {'x': -0.1, 'y': 0.06},'br': {'x': -0.1, 'y': -0.06}
        }

        # Left Side = Normal (1.0), Right Side = Inverted (-1.0)
        self.hip_signs = {
            'bl': 1.0, 'br': -1.0, 
            'fl': -1.0,'fr': 1.0
        }
        self.hip_rot_offsets = {'bl': 0.0, 'br': 0.0, 'fl': 0.0, 'fr': 0.0}

        # --- 2. THIGH CONFIGURATION ---
        self.thigh_motor_sign = {
            'bl': 1.0, 'br': -1.0, 
            'fl': 1.0, 'fr': -1.0
        }
        
        # Geometry Signs (Table Mode: All Positive)
        self.geometry_signs = {
            'bl': 1.0, 'br': 1.0, 
            'fl': 1.0, 'fr': 1.0
        }

        # --- 3. KNEE CONFIGURATION ---
        self.knee_signs = {
            'bl': 1.0, 'br': -1.0, 
            'fl': 1.0, 'fr': -1.0
        }
        self.knee_offsets = {'bl': 0.0, 'br': 0.0, 'fl': 0.0, 'fr': 0.0}

        self.joint_pub = self.create_publisher(
            Float64MultiArray, '/position_controller/commands', 10)
        
        for leg in self.joint_indices.keys():
            self.create_subscription(
                Point, f'/{leg}_foot_target', 
                lambda msg, l=leg: self.foot_callback(msg, l), 10)
        
        self.current_positions = [0.0] * 12
        self.get_logger().info('IK Controller')

    def foot_callback(self, msg: Point, leg_name: str):
        local_x = msg.x - self.hip_offsets[leg_name]['x']
        local_y = msg.y - self.hip_offsets[leg_name]['y']
        local_z = msg.z 
        
        hip, upper, knee = self.calculate_ik(local_x, local_y, local_z, leg_name)
        
        if hip is not None:
            indices = self.joint_indices[leg_name]
            self.current_positions[indices[0]] = hip   
            self.current_positions[indices[1]] = upper 
            self.current_positions[indices[2]] = knee  
            self.publish_joints()

    def calculate_ik(self, x, y, z, leg_name):
        raw_hip = math.atan2(y, abs(z))
        hip_angle = (self.hip_signs[leg_name] * raw_hip) + self.hip_rot_offsets[leg_name]
        
        # --- GEOMETRY ---
        dist = math.sqrt(x**2 + y**2 + z**2)
        max_reach = self.upper_len + self.lower_len
        # Safety clamp
        if dist > max_reach * 100: dist = max_reach * 100 

        # --- 2. KNEE (Elbow) ---
        # Law of Cosines
        cos_knee = (self.upper_len**2 + self.lower_len**2 - dist**2) / \
                   (2 * self.upper_len * self.lower_len)
        cos_knee = np.clip(cos_knee, -1.0, 1.0)
        internal_angle = math.acos(cos_knee)
        
        # Geometric Knee Angle (How much the knee is bent)
        raw_knee = math.pi - internal_angle

        # --- 3. THIGH ---
        swing_angle = math.atan2(x, -z)
        
        cos_beta = (self.upper_len**2 + dist**2 - self.lower_len**2) / \
                   (2 * self.upper_len * dist)
        cos_beta = np.clip(cos_beta, -1.0, 1.0)
        beta = math.acos(cos_beta)
        
        # Geometric Thigh Angle (Swing + Beta)
        raw_upper = swing_angle + (self.geometry_signs[leg_name] * beta)
        
        # --- FINAL MOTOR COMMANDS ---
        
        # Thigh Command
        upper_angle = (self.thigh_motor_sign[leg_name] * raw_upper) + \
                      (self.thigh_motor_sign[leg_name] * self.thigh_offset)

        # Knee Command (WITH PARALLEL LINKAGE CORRECTION)
        # The knee compensates for thigh angle to keep shin properly oriented
        knee_angle = (self.knee_signs[leg_name] * (raw_knee - raw_upper)) + \
                     self.knee_offsets[leg_name]

        return hip_angle, upper_angle, knee_angle

    def publish_joints(self):
        msg = Float64MultiArray()
        msg.data = self.current_positions
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = QuadrupedIKMapped()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()