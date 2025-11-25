#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class FinalCorrectedWalk(Node):
    def __init__(self):
        super().__init__('final_corrected_walk')
        self.pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

        # --- GAIT SETTINGS ---
        self.speed = 4.0            
        self.shoulder_amp = 0.8     
        self.knee_amp = 0.8         
        self.shoulder_offset = 0.0  
        self.knee_offset = 0.0      

        # --- KNEE POLARITY FIX ---
        # Changed all to -1.0 based on your feedback.
        # If one specific side is now wrong (e.g. Left bends up but Right bends down),
        # change ONLY that side back to 1.0.
        self.knee_polarity = {
            'BL': -1.0,  # Back Left
            'BR': -1.0,  # Back Right
            'FL': -1.0,  # Front Left
            'FR': -1.0   # Front Right
        }

        self.timer = self.create_timer(0.02, self.update)
        self.t0 = self.get_clock().now()
        self.get_logger().info("STARTING: Walk with INVERTED Knees")

    def update(self):
        t = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        
        # Master Wave
        wave = math.sin(t * self.speed)

        # 1. SHOULDERS (All Negative, as confirmed previously)
        shoulder_cmd = self.shoulder_offset - (wave * self.shoulder_amp)

        # 2. KNEES (Lift Logic)
        # Lift Group 1 when wave is POSITIVE
        lift_group_1 = max(0, wave) * self.knee_amp
        
        # Lift Group 2 when wave is NEGATIVE
        lift_group_2 = max(0, -wave) * self.knee_amp

        cmd = [0.0] * 12

        # --- GROUP 1: Front Left & Back Right ---
        # FL (Index 6 & 10)
        cmd[6] = shoulder_cmd
        cmd[10] = self.knee_offset + (lift_group_1 * self.knee_polarity['FL'])

        # BR (Index 5 & 9)
        cmd[5] = shoulder_cmd
        cmd[9] = self.knee_offset + (lift_group_1 * self.knee_polarity['BR'])

        # --- GROUP 2: Front Right & Back Left ---
        # FR (Index 7 & 11)
        cmd[7] = shoulder_cmd
        cmd[11] = self.knee_offset + (lift_group_2 * self.knee_polarity['FR'])

        # BL (Index 4 & 8)
        cmd[4] = shoulder_cmd
        cmd[8] = self.knee_offset + (lift_group_2 * self.knee_polarity['BL'])

        self.pub.publish(Float64MultiArray(data=cmd))

def main():
    rclpy.init()
    node = FinalCorrectedWalk()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.pub.publish(Float64MultiArray(data=[0.0]*12))
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()