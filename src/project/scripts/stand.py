#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import numpy as np

class StandUpNode(Node):
    def __init__(self):
        super().__init__('stand_up_commander')
        
        # --- CONFIGURATION ---
        # Target height below the robot body (negative = down)
        self.stand_height = -0.5
        
        # Leg origins relative to robot center (from your config.LEG_ORIGINS)
        # These are where the hip joints are located
        self.leg_origins = {
            'fl': [ 0.1,  0.1],   # Front Left
            'fr': [ 0.1, -0.1],   # Front Right  
            'bl': [-0.1,  0.1],   # Back Left
            'br': [-0.1, -0.1]    # Back Right
        }
        
        # For hip angle = 0, feet should be directly below the hip joints
        # with NO lateral offset (y_offset should be 0)
        self.stance_offset_x = 0.0   # Keep feet directly under hips (no forward/back offset)
        self.stance_offset_y = 0.0   # ZERO lateral offset = hip angle 0
        
        # Create Publishers for each leg target
        self.pubs = {}
        for leg in self.leg_origins:
            topic = f'/{leg}_foot_target'
            self.pubs[leg] = self.create_publisher(Point, topic, 10)
            
        # Timer to keep sending the command (10Hz)
        self.timer = self.create_timer(0.1, self.publish_stand_command)
        
        self.get_logger().info(f"Stand Up Node Started.")
        self.get_logger().info(f"Target height: {self.stand_height}m")
        self.get_logger().info(f"Hip angles will be 0° (feet directly below hips)")
        self.log_foot_positions()

    def publish_stand_command(self):
        for leg, origin in self.leg_origins.items():
            msg = Point()
            
            # For hip angle = 0, foot must be at same Y position as hip
            # X can vary slightly for stability, but Y must match
            
            if leg in ['fl', 'fr']:  # Front legs
                x_offset = self.stance_offset_x
            else:  # Back legs
                x_offset = -self.stance_offset_x
                
            # NO Y OFFSET - this ensures hip angle = 0
            y_offset = self.stance_offset_y
            
            # Body frame position (relative to robot center)
            # Foot Y position = Hip Y position (origin[1]) + 0 offset
            msg.x = float(origin[0] + x_offset)
            msg.y = float(origin[1] + y_offset)  # Same Y as hip = 0° hip angle
            msg.z = float(self.stand_height)
            
            self.pubs[leg].publish(msg)
    
    def log_foot_positions(self):
        """Log the target foot positions once at startup"""
        self.get_logger().info("Target foot positions (body frame):")
        self.get_logger().info("NOTE: Y positions match hip Y → Hip angles = 0°")
        for leg, origin in self.leg_origins.items():
            if leg in ['fl', 'fr']:
                x_offset = self.stance_offset_x
            else:
                x_offset = -self.stance_offset_x
            
            y_offset = self.stance_offset_y
                
            x = origin[0] + x_offset
            y = origin[1] + y_offset
            self.get_logger().info(f"  {leg}: x={x:.3f}, y={y:.3f}, z={self.stand_height:.3f}")
            self.get_logger().info(f"       (Hip at y={origin[1]:.3f}, Foot at y={y:.3f})")

def main(args=None):
    rclpy.init(args=args)
    node = StandUpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()