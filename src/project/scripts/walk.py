#!/usr/bin/env python3
# stable_single_leg_lift.py
# This version leans the body toward the opposite tripod before lifting

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class StableLegLift(Node):
    def __init__(self):
        super().__init__('stable_leg_lift')

        self.pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

        # CHANGE THIS to test any leg safely
        self.leg_to_lift = 'BR'        # Try 'FL', 'FR', 'BL', 'BR' – all stable now

        # Tuned neutral standing pose for your robot (you already found good values)
        self.hip      = 0.0
        self.shoulder = -0.4           # upper leg (negative = forward usually)
        self.knee     = 0.

        # How much we shift the body laterally and rotate to stay balanced
        self.body_shift   = 0.25       # shoulder angle offset for the three support legs
        self.hip_sway     = 0.25       # hip yaw to lean into the triangle

        self.timer = self.create_timer(0.08, self.update)  # ~12.5 Hz
        self.step = 0
        self.get_logger().info(f'Stable leg lift ready – will safely lift {self.leg_to_lift}')

    def update(self):
        t = (self.step % 100) / 50.0                  # 0 → 2 → 0 smooth cycle
        lift = abs(math.sin(t * math.pi))             # 0 → 1 → 0

        # Base neutral pose
        pos = [self.hip]*4 + [self.shoulder]*4 + [self.knee]*4

        # === 1. First, shift body over the supporting tripod ===
        if self.leg_to_lift == 'BR':    # lifting back-right → lean toward FL-FR-BL triangle
            pos[0]  += self.hip_sway      # BL hip yaw in
            pos[2]  += self.hip_sway      # FL hip yaw in
            pos[3]  += self.hip_sway      # FR hip yaw in
            pos[4]  -= self.body_shift    # BL shoulder lean left
            pos[6]  -= self.body_shift    # FL shoulder lean left
            pos[7]  -= self.body_shift    # FR shoulder lean left

        elif self.leg_to_lift == 'BL':
            pos[1] += self.hip_sway       # BR, FR, FL lean right
            pos[3] += self.hip_sway
            pos[2] += self.hip_sway
            pos[5] -= self.body_shift
            pos[7] -= self.body_shift
            pos[6] -= self.body_shift

        elif self.leg_to_lift == 'FR':
            pos[0] += self.hip_sway
            pos[1] += self.hip_sway
            pos[2] += self.hip_sway
            pos[4] -= self.body_shift
            pos[5] -= self.body_shift
            pos[6] -= self.body_shift

        elif self.leg_to_lift == 'FL':
            pos[1] += self.hip_sway
            pos[3] += self.hip_sway
            pos[0] += self.hip_sway
            pos[5] -= self.body_shift
            pos[7] -= self.body_shift
            pos[4] -= self.body_shift

        # === 2. Now safely lift the chosen leg (only when body is shifted) ===
        lift_amount_shoulder = 0.4 * lift
        lift_amount_knee     = 0.7 * lift

        if self.leg_to_lift == 'BR':
            pos[5]  += lift_amount_shoulder   # BR upper leg up
            pos[9]  += lift_amount_knee       # BR knee bend
        elif self.leg_to_lift == 'BL':
            pos[4]  += lift_amount_shoulder
            pos[8]  += lift_amount_knee
        elif self.leg_to_lift == 'FR':
            pos[7]  += lift_amount_shoulder
            pos[11] += lift_amount_knee
        elif self.leg_to_lift == 'FL':
            pos[6]  += lift_amount_shoulder
            pos[10] += lift_amount_knee

        # Publish
        msg = Float64MultiArray()
        msg.data = pos
        self.pub.publish(msg)

        if self.step % 25 == 0:
            self.get_logger().info(f'Lifting {self.leg_to_lift} – height {lift:.2f}')

        self.step += 1

def main():
    rclpy.init()
    node = StableLegLift()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopped")
    finally:
        # Return to perfect neutral on Ctrl+C
        msg = Float64MultiArray()
        msg.data = [0.0]*4 + [-0.4]*4 + [0.6]*4
        node.pub.publish(msg)
        rclpy.shutdown()

if __name__ == '__main__':
    main()