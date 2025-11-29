#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class StandUpNode(Node):
    def __init__(self):
        super().__init__('stand_up_commander')
        
        # --- CONFIGURATION ---
        self.stand_height = -30


        # Where the feet should be relative to the CENTER of the robot.
        # These roughly match your hip offsets so feet are under shoulders.
        self.foot_positions = {
            'fl': [ 0.1,  0.1],  # Front Left
            'fr': [ 0.1, -0.1],  # Front Right
            'bl': [-0.1,  0.1],  # Back Left
            'br': [-0.1, -0.1]   # Back Right
        }
        
        # Create Publishers for each leg target
        self.pubs = {}
        for leg in self.foot_positions:
            topic = f'/{leg}_foot_target'
            self.pubs[leg] = self.create_publisher(Point, topic, 10)
            
        # Timer to keep sending the command (10Hz)
        self.timer = self.create_timer(0.1, self.publish_stand_command)
        self.get_logger().info(f"Stand Up Node Started. Holding height: {self.stand_height}m")

    def publish_stand_command(self):
        for leg, pos in self.foot_positions.items():
            msg = Point()
            
            # Target in BODY FRAME (Relative to center of robot)
            msg.x = float(pos[0])
            msg.y = float(pos[1]) 
            msg.z = float(self.stand_height)
            
            self.pubs[leg].publish(msg)

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