#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorflow as tf
from tensorflow.keras import backend as K
from tensorflow.keras.layers import (Input, ConvLSTM2D, BatchNormalization, Dense,
                                     Flatten, Reshape, RepeatVector, TimeDistributed,
                                     Conv2D, Conv2DTranspose, Lambda)
from tensorflow.keras.models import Model
import os

# --- GPU Configuration ---
os.environ['CUDA_VISIBLE_DEVICES'] = '-1'

# --- MODEL DEFINITION ---
def sampling(args):
    z_mean, z_log_var = args
    batch = K.shape(z_mean)[0]
    dim = K.int_shape(z_mean)[1]
    epsilon = K.random_normal(shape=(batch, dim))
    return z_mean + K.exp(0.5 * z_log_var) * epsilon

def build_vae_model(seq_length=30):
    img_height = 64
    img_width = 64
    channels = 3
    latent_dim = 128
    input_shape = (seq_length, img_height, img_width, channels)
    
    # ENCODER
    encoder_inputs = Input(shape=input_shape)
    x = ConvLSTM2D(filters=16, kernel_size=(3, 3), padding='same', 
                   return_sequences=False, activation='relu')(encoder_inputs)
    x = BatchNormalization()(x)
    x = Conv2D(32, (3, 3), strides=2, padding='same', activation='relu')(x)
    x = BatchNormalization()(x)
    x = Conv2D(64, (3, 3), strides=2, padding='same', activation='relu')(x)
    x = BatchNormalization()(x)
    x = Flatten()(x)
    x = Dense(256, activation='relu')(x)
    
    z_mean = Dense(latent_dim, name='z_mean')(x)
    z_log_var = Dense(latent_dim, name='z_log_var')(x)
    z = Lambda(sampling, output_shape=(latent_dim,), name='z')([z_mean, z_log_var])
    
    encoder = Model(encoder_inputs, [z_mean, z_log_var, z], name='encoder')
    
    # DECODER
    decoder_inputs = Input(shape=(latent_dim,))
    x = RepeatVector(seq_length)(decoder_inputs)
    x = TimeDistributed(Dense(16 * 16 * 64, activation='relu'))(x)
    x = TimeDistributed(Reshape((16, 16, 64)))(x)
    x = TimeDistributed(Conv2DTranspose(32, (3, 3), strides=2, padding='same', activation='relu'))(x)
    x = BatchNormalization()(x)
    x = TimeDistributed(Conv2DTranspose(16, (3, 3), strides=2, padding='same', activation='relu'))(x)
    x = BatchNormalization()(x)
    x = ConvLSTM2D(filters=16, kernel_size=(3, 3), padding='same', 
                   return_sequences=True, activation='relu')(x)
    decoder_outputs = TimeDistributed(Conv2D(3, (3, 3), activation='sigmoid', padding='same'))(x)
    
    decoder = Model(decoder_inputs, decoder_outputs, name='decoder')
    
    # VAE
    vae_outputs = decoder(encoder(encoder_inputs)[2])
    vae = Model(encoder_inputs, vae_outputs, name='vae')
    
    return vae, encoder, decoder


# --- ROS NODE WITH VISUALIZATION ---
class AnomalyDetectorVisual(Node):
    def __init__(self):
        super().__init__('anomaly_detector_visual')
        
        # Parameters
        self.declare_parameter('threshold', 0.005)
        self.declare_parameter('weights_path', '/home/brian/ros2_ws/src/project/data/anomalydetector.weights.h5')
        self.declare_parameter('visualization_enabled', True)
        
        self.seq_length = 30
        self.img_size = (64, 64)
        self.threshold = self.get_parameter('threshold').value
        self.viz_enabled = self.get_parameter('visualization_enabled').value
        self.buffer = []
        
        # Load Model
        self.get_logger().info("Building VAE model...")
        self.model, self.encoder, self.decoder = build_vae_model(self.seq_length)
        
        weights_path = self.get_parameter('weights_path').value
        if os.path.exists(weights_path):
            try:
                self.model.load_weights(weights_path)
                self.get_logger().info("✓ Model weights loaded successfully!")
            except Exception as e:
                self.get_logger().error(f"Failed to load weights: {e}")
                raise
        else:
            self.get_logger().error(f"Weights file not found: {weights_path}")
            raise FileNotFoundError(f"Missing weights: {weights_path}")
        
        # ROS interfaces
        self.bridge = CvBridge()
        
        # Subscribers
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        # Publishers - Core
        self.pub_alert = self.create_publisher(Bool, '/robot/anomaly_detected', 10)
        self.pub_score = self.create_publisher(Float32, '/robot/anomaly_score', 10)
        
        # Publishers - Visualization
        self.pub_overlay = self.create_publisher(Image, '/anomaly/overlay_image', 10)
        self.pub_heatmap = self.create_publisher(Image, '/anomaly/heatmap', 10)
        self.pub_reconstruction = self.create_publisher(Image, '/anomaly/reconstruction', 10)
        self.pub_marker = self.create_publisher(Marker, '/anomaly/marker', 10)
        self.pub_marker_array = self.create_publisher(MarkerArray, '/anomaly/history', 10)
        
        # Anomaly history for visualization
        self.anomaly_history = []
        self.max_history = 50
        
        # Stats
        self.frame_count = 0
        self.anomaly_count = 0
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Anomaly Detector with Visualization Started")
        self.get_logger().info(f"Threshold: {self.threshold}")
        self.get_logger().info(f"Visualization: {'ENABLED' if self.viz_enabled else 'DISABLED'}")
        self.get_logger().info("=" * 60)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return
        
        # Preprocess
        frame = cv2.resize(cv_image, self.img_size)
        frame_normalized = frame.astype('float32') / 255.0
        
        # Buffer management
        self.buffer.append(frame_normalized)
        if len(self.buffer) > self.seq_length:
            self.buffer.pop(0)
        
        # Process when buffer is full
        if len(self.buffer) == self.seq_length:
            self.process_sequence(cv_image, msg.header)
            self.frame_count += 1

    def process_sequence(self, original_image, header):
        # Prepare input
        input_seq = np.array([self.buffer])
        
        # Forward pass
        try:
            reconstruction = self.model.predict(input_seq, verbose=0)
        except Exception as e:
            self.get_logger().error(f"Prediction error: {e}")
            return
        
        # Calculate MSE
        mse = np.mean(np.power(input_seq - reconstruction, 2))
        
        # Publish score
        msg_score = Float32()
        msg_score.data = float(mse)
        self.pub_score.publish(msg_score)
        
        # Check for anomaly
        is_anomaly = mse > self.threshold
        
        # Publish alert
        if is_anomaly:
            self.anomaly_count += 1
            self.get_logger().warn(f"⚠️  ANOMALY! MSE={mse:.6f} (Frame {self.frame_count})")
            
            msg_alert = Bool()
            msg_alert.data = True
            self.pub_alert.publish(msg_alert)
            
            # Add to history
            self.anomaly_history.append({
                'frame': self.frame_count,
                'mse': mse,
                'timestamp': self.get_clock().now()
            })
            if len(self.anomaly_history) > self.max_history:
                self.anomaly_history.pop(0)
        
        # Visualization (if enabled)
        if self.viz_enabled:
            self.publish_visualizations(original_image, input_seq, reconstruction, 
                                       mse, is_anomaly, header)

    def publish_visualizations(self, original_image, input_seq, reconstruction, 
                              mse, is_anomaly, header):
        """
        Publishes multiple visualization formats for RViz and rtabmap
        """
        # 1. OVERLAY IMAGE - Original with anomaly indicator
        overlay = original_image.copy()
        h, w = overlay.shape[:2]
        
        if is_anomaly:
            # Red border for anomaly
            cv2.rectangle(overlay, (0, 0), (w-1, h-1), (0, 0, 255), 10)
            cv2.putText(overlay, f"ANOMALY! MSE: {mse:.4f}", (10, 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        else:
            # Green border for normal
            cv2.rectangle(overlay, (0, 0), (w-1, h-1), (0, 255, 0), 3)
            cv2.putText(overlay, f"Normal - MSE: {mse:.4f}", (10, 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # Add frame counter
        cv2.putText(overlay, f"Frame: {self.frame_count}", (10, h-20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        overlay_msg.header = header
        self.pub_overlay.publish(overlay_msg)
        
        # 2. HEATMAP - Error visualization
        last_frame = input_seq[0, -1]  # Last frame in sequence
        recon_frame = reconstruction[0, -1]  # Corresponding reconstruction
        
        # Per-pixel error
        error = np.abs(last_frame - recon_frame)
        error_gray = np.mean(error, axis=-1)  # Average across RGB
        
        # Normalize and apply colormap
        error_norm = (error_gray * 255).astype(np.uint8)
        heatmap = cv2.applyColorMap(error_norm, cv2.COLORMAP_JET)
        
        # Resize to match original image size
        heatmap_resized = cv2.resize(heatmap, (original_image.shape[1], original_image.shape[0]))
        
        heatmap_msg = self.bridge.cv2_to_imgmsg(heatmap_resized, encoding='bgr8')
        heatmap_msg.header = header
        self.pub_heatmap.publish(heatmap_msg)
        
        # 3. RECONSTRUCTION IMAGE
        recon_frame_uint8 = (recon_frame * 255).astype(np.uint8)
        recon_bgr = cv2.cvtColor(recon_frame_uint8, cv2.COLOR_RGB2BGR)
        recon_resized = cv2.resize(recon_bgr, (original_image.shape[1], original_image.shape[0]))
        
        recon_msg = self.bridge.cv2_to_imgmsg(recon_resized, encoding='bgr8')
        recon_msg.header = header
        self.pub_reconstruction.publish(recon_msg)
        
        # 4. RVIZ MARKER - Visual indicator in 3D space
        if is_anomaly:
            marker = Marker()
            marker.header = header
            marker.header.frame_id = "base_link"  # Adjust to your robot's frame
            marker.ns = "anomaly"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Position above robot
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 1.0
            marker.pose.orientation.w = 1.0
            
            # Size based on severity
            size = 0.3 + (mse * 100)  # Scale with error
            marker.scale.x = min(size, 1.0)
            marker.scale.y = min(size, 1.0)
            marker.scale.z = min(size, 1.0)
            
            # Red, pulsing alpha
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker.lifetime.sec = 2  # Disappear after 2 seconds
            
            self.pub_marker.publish(marker)
        
        # 5. MARKER ARRAY - Anomaly history trail
        marker_array = MarkerArray()
        
        for i, anomaly in enumerate(self.anomaly_history):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "anomaly_history"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Position (you might want to track actual robot position here)
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            # Fade older anomalies
            age = len(self.anomaly_history) - i
            alpha = max(0.2, 1.0 - (age / self.max_history))
            
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = alpha
            
            marker_array.markers.append(marker)
        
        self.pub_marker_array.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = AnomalyDetectorVisual()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"\nFatal error: {e}")
        raise
    finally:
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()