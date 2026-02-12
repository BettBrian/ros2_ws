#!/usr/bin/env python3
"""
Quadruped Inverse Kinematics Controller - Fixed Version
Place in: ros2_ws/src/project/project/scripts/quadruped_ik_controller.py

This version integrates the configuration data properly.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np
from enum import Enum


class LegPosition(Enum):
    """Leg positions on quadruped"""
    FRONT_LEFT = 'fl'
    FRONT_RIGHT = 'fr'
    BACK_LEFT = 'bl'
    BACK_RIGHT = 'br'


class QuadrupedLegIK:
    """
    Inverse Kinematics solver for 3DOF quadruped leg
    
    Link lengths:
    - l1 = 0.057m (Coxa/Hip offset)
    - l2 = 0.130m (Femur/Upper leg)
    - l3 = 0.138m (Tibia/Lower leg)
    """
    
    def __init__(self, l1=0.057, l2=0.130, l3=0.138):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.reach_max = l2 + l3
        self.reach_min = abs(l2 - l3)
    
    def solve(self, x, y, z):
        """
        Solve IK for foot position (x, y, z) in leg frame
        
        Args:
            x: Forward distance (m)
            y: Vertical distance (m, negative = down)
            z: Lateral distance (m, positive = left)
        
        Returns: (theta1, theta2, theta3) in radians or None if unreachable
        """
        # Step 1: Coxa angle (hip abduction/adduction)
        L = np.sqrt(x**2 + z**2)
        theta1 = np.arctan2(z, x)
        
        # Step 2: Project to sagittal plane
        L_prime = L - self.l1
        D = np.sqrt(L_prime**2 + y**2)
        
        # Check reachability
        if D > self.reach_max or D < self.reach_min:
            return None
        
        # Step 3: Knee angle (law of cosines)
        cos_theta3 = (D**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)
        cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)
        theta3 = -np.arccos(cos_theta3)  # Negative for elbow-down
        
        # Step 4: Hip flexion angle
        alpha = np.arctan2(-y, L_prime)  # Note: negative y for downward
        cos_beta = (self.l2**2 + D**2 - self.l3**2) / (2 * self.l2 * D)
        cos_beta = np.clip(cos_beta, -1.0, 1.0)
        beta = np.arccos(cos_beta)
        theta2 = alpha + beta
        
        return (theta1, theta2, theta3)


class QuadrupedController(Node):
    """ROS2 Controller for quadruped robot using inverse kinematics"""
    
    def __init__(self):
        super().__init__('quadruped_ik_controller')
        
        # Robot parameters
        self.l1 = 0.057  # Coxa length (m)
        self.l2 = 0.130  # Femur length (m)
        self.l3 = 0.138  # Tibia length (m)
        
        # IK solver
        self.ik_solver = QuadrupedLegIK(self.l1, self.l2, self.l3)
        
        # --- CONFIGURATION FROM PROVIDED DATA ---
        
        # Joint indices for position array
        self.joint_indices = {
            'bl': [0, 4, 8],   # back left: hip, thigh, knee
            'br': [1, 5, 9],   # back right
            'fl': [2, 6, 10],  # front left
            'fr': [3, 7, 11]   # front right
        }
        
        # Hip offsets from body center
        self.hip_offsets = {
            'fl': {'x': 0.1, 'y': 0.06},   # front left
            'fr': {'x': 0.1, 'y': -0.06},  # front right
            'bl': {'x': -0.1, 'y': 0.06},  # back left
            'br': {'x': -0.1, 'y': -0.06}  # back right
        }
        
        # Hip motor signs (Left = -1.0 for motor, Right = 1.0 for motor)
        # Note: This is inverted from the kinematic sign
        self.hip_signs = {
            'bl': 1.0,   # back left
            'br': 1.0,  # back right
            'fl': 1.0,  # front left
            'fr': 1.0    # front right
        }
        
        self.hip_rot_offsets = {'bl': 0.0, 'br': 0.6436, 'fl': 0.0, 'fr': 0.6436}
        
        # Thigh motor signs
        self.thigh_motor_sign = {
            'bl': -1.0, 
            'br': 1.0, 
            'fl': -1.0, 
            'fr': 1.0
        }
        
        # Geometry signs (Table Mode: All Positive)
        self.geometry_signs = {
            'bl': 1.0, 
            'br': 1.0, 
            'fl': 1.0, 
            'fr': 1.0
        }
        
        # Knee signs
        self.knee_signs = {
            'bl': 1.0, 
            'br': -1.0, 
            'fl': 1.0, 
            'fr': -1.0
        }
        
        self.knee_offsets = {'bl': 0.0, 'br': 0.0, 'fl': 0.0, 'fr': 0.0}
        
        # Joint names mapping (from original code)
        self.joint_names = {
            'fl': {
                'hip': 'flhipmotorplate_link_joint',
                'upper': 'flupperlegmotorplate_link_joint',
                'knee': 'flpart1_link_joint'
            },
            'fr': {
                'hip': 'frhipmotorplate_link_joint',
                'upper': 'frupperlegmotorplate_link_joint',
                'knee': 'frpart1_link_joint'
            },
            'bl': {
                'hip': 'blhipmotorplate_link_joint',
                'upper': 'blupperlegmotorplate_link_joint',
                'knee': 'blpart1_link_joint'
            },
            'br': {
                'hip': 'brhipmotorplate_link_joint',
                'upper': 'brupperlegmotoplate_link_joint',  # Note: typo in original
                'knee': 'brpart1_link_joint'
            }
        }
        
        # Robot dimensions
        self.body_width = 0.12   # m (from hip offsets: 0.06*2)
        self.body_length = 0.20  # m (from hip offsets: 0.1*2)
        self.default_height = -0.20  # m (negative = down)
        
        # Foot positions in leg frame [x, y, z]
        # x: forward, y: down (negative), z: lateral
        self.foot_positions = {
            'fl': [0.15, self.default_height, 0.05],
            'fr': [0.15, self.default_height, 0.05],
            'bl': [0.15, self.default_height, 0.05],
            'br': [0.15, self.default_height, 0.05]
        }
        
        # Gait parameters
        self.gait_phase = 0.0
        self.step_height = 0.03   # m
        self.step_length = 0.04   # m
        self.gait_frequency = 1.0  # Hz
        self.velocity_cmd = [0.0, 0.0, 0.0]  # [vx, vy, omega]
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.command_pub = self.create_publisher(
            Float64MultiArray, '/position_controller/commands', 10
        )
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        
        # Control timer (50 Hz)
        self.dt = 0.02
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info('Quadruped IK Controller initialized')
        self.get_logger().info(f'Links: l1={self.l1}m, l2={self.l2}m, l3={self.l3}m')
        self.get_logger().info(f'Body: {self.body_length}m x {self.body_width}m')
    
    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        self.velocity_cmd[0] = msg.linear.x
        self.velocity_cmd[1] = msg.linear.y
        self.velocity_cmd[2] = msg.angular.z
    
    def trot_gait(self, phase, leg_id):
        """
        Trot gait with diagonal pairs working together for turning
        
        Args:
            phase: Current gait phase (0 to 2π)
            leg_id: Leg identifier ('fl', 'fr', 'bl', 'br')
            
        Returns: (x_offset, y_offset, z_offset)
        """
        # Diagonal pairs: (fl, br) and (fr, bl)
        if leg_id in ['fl', 'br']:
            leg_phase = phase
        else:
            leg_phase = phase + np.pi
        
        hip = self.hip_offsets[leg_id]
        lateral_offset = hip['y']  # +0.06 for left, -0.06 for right
        
        forward_velocity = self.velocity_cmd[0]
        turn_velocity = 0.0
        lateral_swing = 0.0
        
        # Determine if this is a front or back leg
        is_front_leg = leg_id in ['fl', 'fr']
        
        if abs(self.velocity_cmd[2]) > 0.01:
            # Front legs turn more aggressively than back legs
            turn_scale = 0.8 if is_front_leg else 1.0
            
            # Differential drive turning 
            turn_velocity = lateral_offset * self.velocity_cmd[2] * 2.0 * turn_scale
            
            # Hip swing - based on DIAGONAL PAIR, not left/right
            swing_base = 0.04 * turn_scale
            
            
            if leg_id in ['fl', 'br']:  # First diagonal pair
                lateral_swing = swing_base * self.velocity_cmd[2]
            else:  # leg_id in ['fr', 'bl'] - Second diagonal pair
                lateral_swing = -swing_base * self.velocity_cmd[2]
        
        effective_velocity = forward_velocity + turn_velocity
        
        if np.sin(leg_phase) > 0:
            swing_progress = np.sin(leg_phase)
            x_offset = self.step_length * np.cos(leg_phase) * effective_velocity
            y_offset = self.step_height * swing_progress
            
            # Lateral swing: combine strafing + rotation hip swing
            z_offset = (0.02 * swing_progress * self.velocity_cmd[1] + 
                    lateral_swing * swing_progress)
        else:
            x_offset = self.step_length * np.cos(leg_phase) * effective_velocity
            y_offset = 0.0
            z_offset = 0.0
        
        return (x_offset, y_offset, z_offset)
        
    def solve_leg_ik(self, leg_id, foot_pos):
        """
        Solve IK for specific leg with proper sign handling
        
        Args:
            leg_id: Leg identifier ('fl', 'fr', 'bl', 'br')
            foot_pos: [x, y, z] in leg frame
            
        Returns: (theta1, theta2, theta3) with motor signs applied, or None
        """
        x, y, z = foot_pos
        
        # Apply geometry signs for right-side legs (mirror z)
        if leg_id in ['fr', 'br']:
            z_ik = -z
        else:
            z_ik = z
        
        # Solve IK
        result = self.ik_solver.solve(x, y, z_ik)
        
        if result is None:
            return None
        
        theta1, theta2, theta3 = result

        theta2_adjust = theta2 - 1.7
        theta3_adjust = theta3 + 1.16
        theta1_adjust = theta1 - 0.3218
        
        # Apply motor signs and offsets
        hip_angle = self.hip_signs[leg_id] * theta1_adjust + self.hip_rot_offsets[leg_id]
        thigh_angle = self.thigh_motor_sign[leg_id] * self.geometry_signs[leg_id] * theta2_adjust
        knee_angle = self.knee_signs[leg_id] * theta3_adjust + self.knee_offsets[leg_id]
        
        return (hip_angle, thigh_angle, knee_angle)
    
    def control_loop(self):
        """Main control loop - called at 50 Hz"""
        # Update gait phase
        if abs(self.velocity_cmd[0]) > 0.01 or abs(self.velocity_cmd[1]) > 0.01 or abs(self.velocity_cmd[2]) > 0.01:
            self.gait_phase += 2 * np.pi * self.gait_frequency * self.dt
        else:
            self.gait_phase = 0.0
        
        self.gait_phase = self.gait_phase % (2 * np.pi)
        
        # Prepare output arrays (12 joints total)
        positions = [0.0] * 12
        all_names = [''] * 12
        success = True
        
        for leg_id in ['bl', 'br', 'fl', 'fr']:
            # Get gait offset
            if abs(self.velocity_cmd[0]) > 0.01 or abs(self.velocity_cmd[1]) > 0.01 or abs(self.velocity_cmd[2]) > 0.01:

                offset = self.trot_gait(self.gait_phase, leg_id)
            else:
                offset = (0.0, 0.0, 0.0)
            
            # Target foot position
            base_pos = self.foot_positions[leg_id]
            target_pos = [
                base_pos[0] + offset[0],
                base_pos[1] + offset[1],
                base_pos[2] + offset[2]
            ]
            
            # Solve IK
            angles = self.solve_leg_ik(leg_id, target_pos)
            
            if angles is not None:
                # Get joint indices for this leg
                indices = self.joint_indices[leg_id]
                
                # Assign to position array
                positions[indices[0]] = angles[0]  # hip
                positions[indices[1]] = angles[1]  # thigh
                positions[indices[2]] = angles[2]  # knee
                
                # Assign joint names
                leg_joints = self.joint_names[leg_id]
                all_names[indices[0]] = leg_joints['hip']
                all_names[indices[1]] = leg_joints['upper']
                all_names[indices[2]] = leg_joints['knee']
            else:
                self.get_logger().warn(
                    f'IK failed for {leg_id} at pos {target_pos}',
                    throttle_duration_sec=1.0
                )
                success = False
        
        # Only publish if all legs succeeded
        if success:
            # Publish joint states
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = all_names
            joint_state.position = positions
            self.joint_pub.publish(joint_state)
            
            # Publish to position controller
            cmd_msg = Float64MultiArray()
            cmd_msg.data = positions
            self.command_pub.publish(cmd_msg)
    
    def set_body_height(self, height):
        """Adjust robot body height"""
        self.default_height = height
        for leg_id in ['fl', 'fr', 'bl', 'br']:
            self.foot_positions[leg_id][1] = height
    
    def stand_neutral(self):
        """Return to neutral standing position"""
        self.velocity_cmd = [0.0, 0.0, 0.0]
        for leg_id in ['fl', 'fr', 'bl', 'br']:
            self.foot_positions[leg_id] = [0.15, self.default_height, 0.05]


def main(args=None):
    rclpy.init(args=args)
    controller = QuadrupedController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()