#!/usr/bin/env python3
"""
Quadruped Calibration and Diagnostic Tool
Place in: ros2_ws/src/project/project/scripts/calibrate_robot.py

This script helps diagnose why the robot is collapsed and provides
step-by-step calibration to fix motor signs and angles.

Usage: ros2 run project calibrate_robot
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import sys
import time


class QuadrupedCalibration(Node):
    """Calibration tool for quadruped robot"""
    
    def __init__(self):
        super().__init__('quadruped_calibration')
        
        # Publisher for joint commands
        self.command_pub = self.create_publisher(
            Float64MultiArray, '/position_controller/commands', 10
        )
        
        # Joint indices
        self.joint_indices = {
            'bl': [0, 4, 8],
            'br': [1, 5, 9],
            'fl': [2, 6, 10],
            'fr': [3, 7, 11]
        }
        
        # Current motor signs (from your config)
        self.hip_signs = {'bl': 1.0, 'br': -1.0, 'fl': -1.0, 'fr': 1.0}
        self.thigh_signs = {'bl': 1.0, 'br': -1.0, 'fl': 1.0, 'fr': -1.0}
        self.knee_signs = {'bl': 1.0, 'br': -1.0, 'fl': 1.0, 'fr': -1.0}
        
        # Link lengths
        self.l1 = 0.057
        self.l2 = 0.130
        self.l3 = 0.138
        
        self.get_logger().info('Quadruped Calibration Tool Ready')
    
    def send_command(self, positions):
        """Send joint position command"""
        msg = Float64MultiArray()
        msg.data = positions
        self.command_pub.publish(msg)
        time.sleep(0.1)  # Allow message to be sent
    
    def zero_all_joints(self):
        """Set all joints to zero position"""
        print("\n" + "="*60)
        print("ZEROING ALL JOINTS")
        print("="*60)
        print("All motors will be set to 0 radians.")
        print("Expected behavior: Robot should be in a neutral/reference pose")
        
        input("Press ENTER to zero all joints...")
        
        positions = [0.0] * 12
        self.send_command(positions)
        
        print("\n✓ All joints set to 0.0 radians")
        print("\nOBSERVE YOUR ROBOT:")
        print("- Are legs pointing straight out from body?")
        print("- Are knees unbent (straight)?")
        print("- Does the pose look symmetric?")
        input("\nPress ENTER to continue...")
    
    def test_standing_angles(self):
        """Calculate and test standing position angles"""
        print("\n" + "="*60)
        print("TESTING STANDING POSITION")
        print("="*60)
        
        # Calculate standing angles using IK
        # Target: foot at (x=0.15, y=-0.20, z=0.05) in leg frame
        x, y, z = 0.15, -0.20, 0.05
        
        print(f"\nTarget foot position (leg frame):")
        print(f"  x = {x:6.3f}m (forward from hip)")
        print(f"  y = {y:6.3f}m (down from hip)")
        print(f"  z = {z:6.3f}m (lateral from hip)")
        
        # Solve IK
        L = np.sqrt(x**2 + z**2)
        theta1 = np.arctan2(z, x)
        
        L_prime = L - self.l1
        D = np.sqrt(L_prime**2 + y**2)
        
        cos_theta3 = (D**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)
        cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)
        theta3 = -np.arccos(cos_theta3)
        
        alpha = np.arctan2(-y, L_prime)
        cos_beta = (self.l2**2 + D**2 - self.l3**2) / (2 * self.l2 * D)
        cos_beta = np.clip(cos_beta, -1.0, 1.0)
        beta = np.arccos(cos_beta)
        theta2 = alpha + beta
        
        print(f"\nCalculated angles (NO motor signs applied):")
        print(f"  θ1 (hip)   = {np.rad2deg(theta1):>7.2f}° ({theta1:>7.4f} rad)")
        print(f"  θ2 (thigh) = {np.rad2deg(theta2):>7.2f}° ({theta2:>7.4f} rad)")
        print(f"  θ3 (knee)  = {np.rad2deg(theta3):>7.2f}° ({theta3:>7.4f} rad)")
        
        print("\n" + "-"*60)
        print("TESTING EACH LEG INDIVIDUALLY")
        print("-"*60)
        
        for leg_id in ['fl', 'fr', 'bl', 'br']:
            print(f"\n>>> Testing {leg_id.upper()} leg <<<")
            
            # Apply motor signs for this leg
            if leg_id in ['fr', 'br']:
                z_ik = -z
                theta1_ik = np.arctan2(z_ik, x)
            else:
                theta1_ik = theta1
            
            hip_cmd = self.hip_signs[leg_id] * theta1_ik
            thigh_cmd = self.thigh_signs[leg_id] * theta2
            knee_cmd = self.knee_signs[leg_id] * theta3
            
            print(f"  Motor commands (with signs):")
            print(f"    Hip:   {hip_cmd:>7.4f} rad ({np.rad2deg(hip_cmd):>7.2f}°)")
            print(f"    Thigh: {thigh_cmd:>7.4f} rad ({np.rad2deg(thigh_cmd):>7.2f}°)")
            print(f"    Knee:  {knee_cmd:>7.4f} rad ({np.rad2deg(knee_cmd):>7.2f}°)")
            
            input(f"\n  Press ENTER to move {leg_id.upper()} to standing position...")
            
            # Send command for this leg only
            positions = [0.0] * 12
            indices = self.joint_indices[leg_id]
            positions[indices[0]] = hip_cmd
            positions[indices[1]] = thigh_cmd
            positions[indices[2]] = knee_cmd
            
            self.send_command(positions)
            
            print(f"\n  OBSERVE {leg_id.upper()} leg:")
            print(f"    - Should foot be on ground at ~20cm below hip?")
            print(f"    - Should leg be extended forward ~15cm?")
            print(f"    - Should knee be bent (not straight)?")
            
            response = input(f"\n  Does {leg_id.upper()} look correct? (y/n): ").lower()
            
            if response != 'y':
                print(f"\n  ⚠️  {leg_id.upper()} needs adjustment!")
                print(f"  Current signs: Hip={self.hip_signs[leg_id]}, "
                      f"Thigh={self.thigh_signs[leg_id]}, "
                      f"Knee={self.knee_signs[leg_id]}")
                
                print(f"\n  Common fixes:")
                print(f"    - If leg extends up instead of down: flip thigh sign")
                print(f"    - If knee bends wrong way: flip knee sign")
                print(f"    - If hip rotates wrong way: flip hip sign")
    
    def test_all_standing(self):
        """Test all legs in standing position simultaneously"""
        print("\n" + "="*60)
        print("TESTING ALL LEGS STANDING")
        print("="*60)
        
        x, y, z = 0.15, -0.20, 0.05
        
        positions = [0.0] * 12
        
        for leg_id in ['fl', 'fr', 'bl', 'br']:
            # Solve IK
            if leg_id in ['fr', 'br']:
                z_ik = -z
            else:
                z_ik = z
            
            L = np.sqrt(x**2 + z_ik**2)
            theta1 = np.arctan2(z_ik, x)
            
            L_prime = L - self.l1
            D = np.sqrt(L_prime**2 + y**2)
            
            cos_theta3 = (D**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)
            cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)
            theta3 = -np.arccos(cos_theta3)
            
            alpha = np.arctan2(-y, L_prime)
            cos_beta = (self.l2**2 + D**2 - self.l3**2) / (2 * self.l2 * D)
            cos_beta = np.clip(cos_beta, -1.0, 1.0)
            beta = np.arccos(cos_beta)
            theta2 = alpha + beta
            
            # Apply motor signs
            hip_cmd = self.hip_signs[leg_id] * theta1
            thigh_cmd = self.thigh_signs[leg_id] * theta2
            knee_cmd = self.knee_signs[leg_id] * theta3
            
            # Set positions
            indices = self.joint_indices[leg_id]
            positions[indices[0]] = hip_cmd
            positions[indices[1]] = thigh_cmd
            positions[indices[2]] = knee_cmd
        
        input("\nPress ENTER to move ALL legs to standing position...")
        
        self.send_command(positions)
        
        print("\n✓ All legs commanded to standing position")
        print("\nOBSERVE YOUR ROBOT:")
        print("  - Should robot be standing on all four feet?")
        print("  - Should body be roughly level (horizontal)?")
        print("  - Should body be about 20cm above ground?")
        print("  - Should stance look stable and balanced?")
        
        response = input("\nIs robot standing correctly? (y/n): ").lower()
        
        if response == 'y':
            print("\n🎉 SUCCESS! Robot is standing correctly!")
            print("\nYour motor signs are correct. You can now use the controller.")
        else:
            print("\n⚠️  Robot not standing correctly.")
            print("\nLikely issues:")
            print("  1. One or more motor signs are wrong")
            print("  2. Link lengths don't match your robot")
            print("  3. Joint limits preventing correct pose")
            print("\nRun individual leg tests above to diagnose.")
    
    def test_single_joint(self):
        """Test individual joint movements"""
        print("\n" + "="*60)
        print("SINGLE JOINT TESTING")
        print("="*60)
        
        legs = ['fl', 'fr', 'bl', 'br']
        joints = ['hip', 'thigh', 'knee']
        
        print("\nAvailable tests:")
        for i, leg in enumerate(legs):
            for j, joint in enumerate(joints):
                test_num = i * 3 + j + 1
                print(f"  {test_num:2d}. {leg.upper()} {joint}")
        print("   0. Back to main menu")
        
        choice = input("\nSelect test (0-12): ")
        
        try:
            choice = int(choice)
            if choice == 0:
                return
            
            if 1 <= choice <= 12:
                leg_idx = (choice - 1) // 3
                joint_idx = (choice - 1) % 3
                
                leg_id = legs[leg_idx]
                joint_name = joints[joint_idx]
                
                print(f"\nTesting {leg_id.upper()} {joint_name}")
                print("Will move joint through +0.5 rad, 0.0 rad, -0.5 rad")
                
                for angle in [0.5, 0.0, -0.5, 0.0]:
                    input(f"\nPress ENTER to move to {angle:+.1f} rad...")
                    
                    positions = [0.0] * 12
                    indices = self.joint_indices[leg_id]
                    positions[indices[joint_idx]] = angle
                    
                    self.send_command(positions)
                    
                    print(f"  Commanded: {angle:+.2f} rad")
                    print(f"  Observe: Which direction did the joint move?")
                
                print("\n✓ Test complete")
        
        except ValueError:
            print("Invalid choice")
    
    def run_calibration(self):
        """Main calibration menu"""
        while True:
            print("\n" + "█"*60)
            print("█" + " "*58 + "█")
            print("█" + "  QUADRUPED CALIBRATION TOOL".center(58) + "█")
            print("█" + " "*58 + "█")
            print("█"*60)
            
            print("\nMain Menu:")
            print("  1. Zero all joints")
            print("  2. Test standing position (all legs)")
            print("  3. Test individual legs")
            print("  4. Test single joint")
            print("  5. Print current configuration")
            print("  0. Exit")
            
            choice = input("\nSelect option: ")
            
            try:
                choice = int(choice)
                
                if choice == 0:
                    print("\nExiting calibration tool...")
                    break
                elif choice == 1:
                    self.zero_all_joints()
                elif choice == 2:
                    self.test_all_standing()
                elif choice == 3:
                    self.test_standing_angles()
                elif choice == 4:
                    self.test_single_joint()
                elif choice == 5:
                    self.print_config()
                else:
                    print("Invalid option")
            
            except ValueError:
                print("Invalid input")
            except KeyboardInterrupt:
                print("\n\nInterrupted by user")
                break
    
    def print_config(self):
        """Print current configuration"""
        print("\n" + "="*60)
        print("CURRENT CONFIGURATION")
        print("="*60)
        
        print("\nLink lengths:")
        print(f"  l1 (coxa):  {self.l1} m")
        print(f"  l2 (femur): {self.l2} m")
        print(f"  l3 (tibia): {self.l3} m")
        
        print("\nMotor signs:")
        print("  Leg  | Hip  | Thigh | Knee")
        print("  -----|------|-------|------")
        for leg in ['fl', 'fr', 'bl', 'br']:
            print(f"  {leg.upper()}  | {self.hip_signs[leg]:>+4.1f} | {self.thigh_signs[leg]:>+5.1f} | {self.knee_signs[leg]:>+4.1f}")
        
        print("\nJoint indices (position in command array):")
        print("  Leg  | Hip | Thigh | Knee")
        print("  -----|-----|-------|------")
        for leg in ['fl', 'fr', 'bl', 'br']:
            idx = self.joint_indices[leg]
            print(f"  {leg.upper()}  | [{idx[0]:2d}] | [{idx[1]:2d}]  | [{idx[2]:2d}]")
        
        input("\nPress ENTER to continue...")


def main(args=None):
    rclpy.init(args=args)
    
    calibrator = QuadrupedCalibration()
    
    try:
        # Give ROS time to initialize
        time.sleep(1.0)
        
        print("\n⚠️  SAFETY NOTICE:")
        print("  - Ensure robot is securely supported or on soft surface")
        print("  - Be ready to power off if needed")
        print("  - Start with gentle movements")
        
        input("\nPress ENTER when ready to begin calibration...")
        
        calibrator.run_calibration()
        
    except KeyboardInterrupt:
        print("\n\nCalibration interrupted")
    finally:
        # Return to zero
        print("\nReturning all joints to zero...")
        msg = Float64MultiArray()
        msg.data = [0.0] * 12
        calibrator.command_pub.publish(msg)
        time.sleep(0.5)
        
        calibrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()