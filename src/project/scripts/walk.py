#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import math

class SlowWalk(Node):
    def __init__(self):
        super().__init__('slow_walk_commander')
        
        
        # --- GAIT SETTINGS ---
        self.stand_height = -0.5  # height
        self.lift_height = 0.4   #  lift
        self.stride_length = 0.3  # step
        
        self.speed = 0.2           
        
        # GAIT PATTERN: Crawl
        self.swing_ratio = 0.5
        
        # --- FOOT HOME POSITIONS ---
        self.foot_home = {
            'fl': {'x': 0.1,  'y': 0.06},
            'fr': {'x': 0.1,  'y': -0.06},
            'bl': {'x': -0.1, 'y': 0.06},
            'br': {'x': -0.1, 'y': -0.06}
        }
        
        # --- TIMING OFFSETS ---
        self.phase_offsets = {
            'fl': 0.0,
            'br': 0.25,
            'fr': 0.50,
            'bl': 0.75
        }

        self.pubs = {}
        for leg in self.foot_home:
            topic = f'/{leg}_foot_target'
            self.pubs[leg] = self.create_publisher(Point, topic, 10)

        # Use simulation time
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.update_gait)
        
        self.get_logger().info("="*50)
        self.get_logger().info(f"WALK")
        self.get_logger().info(f"Stand: {self.stand_height}m, Lift: {self.lift_height}m")
        self.get_logger().info(f"Stride: {self.stride_length}m, Speed: {self.speed} Hz")
        self.get_logger().info("="*50)

    def get_trajectory(self, phase, home_x, home_y):
        half_stride = self.stride_length / 2
        
        # --- SWING PHASE (Air) ---
        if phase < self.swing_ratio:
            swing_prog = phase / self.swing_ratio
            
            # X: Move from BACK to FRONT (Cosine for smooth acceleration)
            x_pos = home_x - (math.cos(math.pi * swing_prog) * half_stride)
            
            # Z: Lift UP (Sine Arch)
            z_pos = self.stand_height + (math.sin(math.pi * swing_prog) * self.lift_height)
            
        # --- STANCE PHASE (Ground) ---
        else:
            stance_prog = (phase - self.swing_ratio) / (1.0 - self.swing_ratio)
            
            # X: Move from FRONT to BACK (Linear slide)
            x_pos = home_x + half_stride - (stance_prog * self.stride_length)
            
            # Z: Flat on ground
            z_pos = self.stand_height
            
        return x_pos, home_y, z_pos

    def update_gait(self):
        # Use simulation time
        current_time = self.get_clock().now()
        elapsed_sec = (current_time - self.start_time).nanoseconds / 1e9
        
        # Global cycle (0.0 to 1.0 loops continuously)
        cycle_progress = (elapsed_sec * self.speed) % 1.0
        
        for leg, home in self.foot_home.items():
            # Calculate local phase for this leg
            leg_phase = (cycle_progress + self.phase_offsets[leg]) % 1.0
            
            # Get Target
            x, y, z = self.get_trajectory(leg_phase, home['x'], home['y'])
            
            # Publish
            msg = Point()
            msg.x = float(x)
            msg.y = float(y)
            msg.z = float(z)
            self.pubs[leg].publish(msg)

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