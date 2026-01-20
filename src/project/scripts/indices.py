#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard

class ObjectModeTeleop(Node):
    def __init__(self):
        super().__init__('object_teleop')
        # This usually targets the robot's base controller
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.lin_speed = 0.5  # m/s
        self.ang_speed = 1.0  # rad/s
        self.pressed_keys = set()
        
        self.get_logger().info("OBJECT TRANSLATION MODE: Use WASD (Move) and QE (Rotate)")
        
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        self.timer = self.create_timer(0.05, self.publish_cmd)

    def on_press(self, key):
        try: self.pressed_keys.add(key.char.lower())
        except: pass

    def on_release(self, key):
        try: self.pressed_keys.remove(key.char.lower())
        except: pass

    def publish_cmd(self):
        msg = Twist()
        # Translation (Move like an object)
        if 'w' in self.pressed_keys: msg.linear.x = self.lin_speed
        if 's' in self.pressed_keys: msg.linear.x = -self.lin_speed
        if 'a' in self.pressed_keys: msg.linear.y = self.lin_speed
        if 'd' in self.pressed_keys: msg.linear.y = -self.lin_speed
        
        # Rotation (Spin like an object)
        if 'q' in self.pressed_keys: msg.angular.z = self.ang_speed
        if 'e' in self.pressed_keys: msg.angular.z = -self.ang_speed
        
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = ObjectModeTeleop()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__':
    main()