#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import math

class SlowWalk(Node):
    def __init__(self):
        super().__init__('slow_walk_commander')
        
        # --- GAIT SETTINGS ---
        self.stand_height = -0.8   # REALISTIC height (was -0.8, way too low!)
        self.lift_height = 0.6    # 4cm lift (was 0.55, way too high!)
        self.stride_length = 0.3   # 8cm stride (was 0.4, too long!)
        
        self.speed = 0.3            # Hz - gait frequency
        
        # GAIT PATTERN: Trot (diagonal pairs)
        self.swing_ratio = 0.5
        
        # --- FOOT HOME POSITIONS ---
        self.foot_home = {
            'fl': {'x': 0.1,  'y': 0.06},
            'fr': {'x': 0.1,  'y': -0.06},
            'bl': {'x': -0.1, 'y': 0.06},
            'br': {'x': -0.1, 'y': -0.06}
        }
        
        # --- TIMING OFFSETS (Trot: diagonal pairs) ---
        self.phase_offsets = {
            'fl': 0.0,
            'br': 0.0,
            'fr': 0.5,
            'bl': 0.5
        }

        self.pubs = {}
        for leg in self.foot_home:
            topic = f'/{leg}_foot_target'
            self.pubs[leg] = self.create_publisher(Point, topic, 10)

        # Use simulation time
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.update_gait)
        
        self.get_logger().info("="*60)
        self.get_logger().info("TROT WALK - CORRECTED FOR FORWARD MOTION")
        self.get_logger().info(f"Stand: {self.stand_height}m, Lift: {self.lift_height}m")
        self.get_logger().info(f"Stride: {self.stride_length}m, Speed: {self.speed} Hz")
        self.get_logger().info("="*60)
        self.get_logger().info("Front legs: push BACKWARD → robot moves FORWARD")
        self.get_logger().info("Back legs:  push FORWARD  → robot moves FORWARD")
        self.get_logger().info("="*60)

    def smooth_step(self, t):
        """Smoothstep function"""
        return t * t * (3.0 - 2.0 * t)

    def get_trajectory(self, phase, home_x, home_y, is_front_leg):
        """
        Calculate foot trajectory
        
        CRITICAL: Front and back legs need OPPOSITE stride directions!
        - Front legs: During stance, move from FRONT to BACK (push backward)
        - Back legs: During stance, move from BACK to FRONT (push forward)
        Both create FORWARD motion for the robot!
        """
        half_stride = self.stride_length / 2
        
        # --- SWING PHASE (Foot in air) ---
        if phase < self.swing_ratio:
            swing_prog = phase / self.swing_ratio
            smooth_prog = self.smooth_step(swing_prog)
            
            if is_front_leg:
                # Front leg swing: BACK → FRONT
                x_pos = home_x - half_stride + (smooth_prog * self.stride_length)
            else:
                # Back leg swing: FRONT → BACK
                x_pos = home_x + half_stride - (smooth_prog * self.stride_length)
            
            # Z: Parabolic lift
            z_pos = self.stand_height + (math.sin(math.pi * swing_prog) * self.lift_height)
            
        # --- STANCE PHASE (Foot on ground, pushing) ---
        else:
            stance_prog = (phase - self.swing_ratio) / (1.0 - self.swing_ratio)
            smooth_prog = self.smooth_step(stance_prog)
            
            if is_front_leg:
                # Front leg stance: FRONT → BACK (pushes robot forward)
                x_pos = home_x + half_stride - (smooth_prog * self.stride_length)
            else:
                # Back leg stance: BACK → FRONT (pushes robot forward)
                x_pos = home_x - half_stride + (smooth_prog * self.stride_length)
            
            # Z: Flat on ground
            z_pos = self.stand_height
            
        return x_pos, home_y, z_pos

    def update_gait(self):
        """Main gait update loop"""
        current_time = self.get_clock().now()
        elapsed_sec = (current_time - self.start_time).nanoseconds / 1e9
        
        # Global cycle (0.0 to 1.0 loops continuously)
        cycle_progress = (elapsed_sec * self.speed) % 1.0
        
        for leg, home in self.foot_home.items():
            # Determine if this is a front leg
            is_front = leg in ['fl', 'fr']
            
            # Calculate local phase for this leg
            leg_phase = (cycle_progress + self.phase_offsets[leg]) % 1.0
            
            # Get trajectory (with correct direction for front/back)
            x, y, z = self.get_trajectory(leg_phase, home['x'], home['y'], is_front)
            
            # Publish
            msg = Point()
            msg.x = float(x)
            msg.y = float(y)
            msg.z = float(z)
            self.pubs[leg].publish(msg)
        
        # Debug logging
        if int(elapsed_sec / 2) != getattr(self, '_last_log', -1):
            self._last_log = int(elapsed_sec / 2)
            
            # Check which legs are in swing
            fl_phase = (cycle_progress + self.phase_offsets['fl']) % 1.0
            fr_phase = (cycle_progress + self.phase_offsets['fr']) % 1.0
            
            if fl_phase < self.swing_ratio:
                state = "FL+BR SWING, FR+BL STANCE"
            else:
                state = "FR+BL SWING, FL+BR STANCE"
            
            self.get_logger().info(f"Cycle: {cycle_progress:.2f} | {state}")

def main(args=None):
    rclpy.init(args=args)
    node = SlowWalk()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()