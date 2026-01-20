import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoRecorder(Node):
    def __init__(self):
        super().__init__('video_recorder')
        self.bridge = CvBridge()
        
        # Video writer setup
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # or 'XVID' for .avi
        self.out = cv2.VideoWriter('camera_recording.mp4', fourcc, 30.0, (640, 480))
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        self.frame_count = 0
        self.get_logger().info('Recording started...')
    
    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Write frame to video
        self.out.write(cv_image)
        self.frame_count += 1
        
        if self.frame_count % 30 == 0:  # Log every 30 frames
            self.get_logger().info(f'Recorded {self.frame_count} frames')
    
    def __del__(self):
        self.out.release()
        self.get_logger().info(f'Video saved! Total frames: {self.frame_count}')

def main():
    rclpy.init()
    recorder = VideoRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    
    recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()