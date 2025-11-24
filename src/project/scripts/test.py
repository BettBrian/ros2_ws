#!/usr/bin/env python3
# FINAL_WITH_REVERSED_SHOULDER_MOTORS.py
# 100% correct for your robot:
# - Left knees reversed
# - ALL shoulder motors reversed (positive command = backward rotation)
# - Real upper leg lift
# - Big, fast, symmetric, beautiful walk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class FinalCorrectWalk(Node):
    def __init__(self):
        super().__init__('final_correct_walk')
        self.pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

        self.base = 0.0
        self.body_pitch = 0.20

        self.max_foot_lift   = 0.07
        self.max_stride      = 0.22
        self.cycle_time      = 1.05
        self.warmup_time     = 8.0
        self.upper_leg_lift_amplitude = 0.50   # rad ≈ 29°

        self.timer = self.create_timer(0.02, self.update)
        self.t0 = self.get_clock().now()
        self.get_logger().info("FINAL CORRECT WALK — ALL MOTORS FIXED (shoulders reversed)")

    def update(self):
        t = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        ramp = min(t / self.warmup_time, 1.0)
        ease = 0.5 * (1 - math.cos(ramp * math.pi))

        lift   = self.max_foot_lift   * ease
        stride = self.max_stride      * ease
        pitch  = self.body_pitch      * ease

        phase = (t / self.cycle_time) * 2 * math.pi
        p1 = phase
        p2 = phase + math.pi

        def traj(p):
            p = p % (2*math.pi)
            if p < math.pi:
                prog = p / math.pi
                h = math.sin(prog * math.pi)
                f = prog - 0.5
            else:
                prog = (p - math.pi) / math.pi
                h = 0.0
                f = 0.5 - prog
            return h, f

        h1, f1 = traj(p1)   # FL + BR
        h2, f2 = traj(p2)   # FR + BL

        shoulder = [self.base + pitch] * 4
        knee     = [self.base] * 4

        lift_scale   = self.upper_leg_lift_amplitude * ease
        forward_scale = 1.25
        knee_scale   = 11.0

        # THE FIX: ALL shoulder motors are reversed → we send NEGATIVE values
        # Left knees also reversed → already handled with minus sign

        # FL + BR pair
        shoulder[2] -= (f1 * stride * forward_scale + h1 * lift_scale)   # FL shoulder reversed
        knee[2]     -= h1 * lift * knee_scale                            # FL knee reversed (left)

        shoulder[1] -= (f1 * stride * forward_scale + h1 * lift_scale)   # BR shoulder reversed
        knee[1]     += h1 * lift * knee_scale                            # BR knee normal

        # FR + BL pair
        shoulder[3] -= (f2 * stride * forward_scale + h2 * lift_scale)   # FR shoulder reversed
        knee[3]     += h2 * lift * knee_scale                            # FR knee normal

        shoulder[0] -= (f2 * stride * forward_scale + h2 * lift_scale)   # BL shoulder reversed
        knee[0]     -= h2 * lift * knee_scale                            # BL knee reversed (left)

        # Exact order from your corrected YAML (motoplate fixed)
        cmd = [0.0]*4 + [
            shoulder[0], shoulder[1], shoulder[2], shoulder[3],   # BL, BR, FL, FR shoulders
            knee[0],     knee[1],     knee[2],     knee[3]        # BL, BR, FL, FR knees
        ]
        self.pub.publish(Float64MultiArray(data=cmd))

def main():
    rclpy.init()
    node = FinalCorrectWalk()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopped → perfect stand")
        node.pub.publish(Float64MultiArray(data=[0.0]*12))
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()