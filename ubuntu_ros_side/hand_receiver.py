#!/usr/bin/env python3
"""
ROS2 Hand Tracking Receiver Node
Receives UDP hand data and publishes to ROS2 topics for robot control
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3, PoseStamped
from std_msgs.msg import Float32, Header
import socket
import json
import threading
import time
from typing import Dict, List, Optional

class HandReceiver(Node):
    def __init__(self):
        super().__init__('hand_receiver')
        
        # Configuration
        self.listen_port = 9999
        self.socket = None
        self.running = False
        
        # Statistics
        self.packet_count = 0
        self.last_packet_time = 0
        self.start_time = time.time()
        
        # ROS2 Publishers
        self.left_palm_pub = self.create_publisher(PoseStamped, '/hand_tracking/left_palm', 10)
        self.right_palm_pub = self.create_publisher(PoseStamped, '/hand_tracking/right_palm', 10)
        self.left_grab_pub = self.create_publisher(Float32, '/hand_tracking/left_grab_strength', 10)
        self.right_grab_pub = self.create_publisher(Float32, '/hand_tracking/right_grab_strength', 10)
        
        # Timer for statistics
        self.stats_timer = self.create_timer(2.0, self.print_stats)
        
        self.get_logger().info("Hand Receiver Node initialized")
        self.get_logger().info(f"Listening on UDP port {self.listen_port}")
        
        # Start UDP listener in separate thread
        self.start_udp_listener()
    
    def start_udp_listener(self):
        """Start UDP socket listener in background thread"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind(('', self.listen_port))
            self.socket.settimeout(1.0)
            self.running = True
            
            self.listener_thread = threading.Thread(target=self.udp_listener_loop)
            self.listener_thread.daemon = True
            self.listener_thread.start()
            
            self.get_logger().info("UDP listener started successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to start UDP listener: {e}")
    
    def udp_listener_loop(self):
        """Main UDP receiving loop"""
        while self.running:
            try:
                data, addr = self.socket.recvfrom(4096)
                self.process_hand_data(data.decode('utf-8'))
                
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"UDP receive error: {e}")
    
    def process_hand_data(self, json_data: str):
        """Process received hand tracking data"""
        try:
            data = json.loads(json_data)
            self.packet_count += 1
            self.last_packet_time = time.time()
            
            for hand in data.get('hands', []):
                self.publish_hand_data(hand, data['timestamp'])
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decode error: {e}")
        except Exception as e:
            self.get_logger().error(f"Data processing error: {e}")
    
    def publish_hand_data(self, hand: Dict, timestamp: float):
        """Publish individual hand data to ROS2 topics"""
        hand_type = hand.get('type', 'unknown')
        palm = hand.get('palm', {})
        position = palm.get('position', [0, 0, 0])
        direction = palm.get('direction', [0, 0, 1])
        normal = palm.get('normal', [0, 1, 0])
        
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = f"{hand_type}_hand"
        
        pose_msg.pose.position.x = position[0] / 1000.0
        pose_msg.pose.position.y = position[1] / 1000.0
        pose_msg.pose.position.z = position[2] / 1000.0
        
        pose_msg.pose.orientation.x = direction[0]
        pose_msg.pose.orientation.y = direction[1]
        pose_msg.pose.orientation.z = direction[2]
        pose_msg.pose.orientation.w = 1.0
        
        grab_msg = Float32()
        grab_msg.data = float(hand.get('grab_strength', 0.0))
        
        if hand_type == 'left':
            self.left_palm_pub.publish(pose_msg)
            self.left_grab_pub.publish(grab_msg)
        elif hand_type == 'right':
            self.right_palm_pub.publish(pose_msg)
            self.right_grab_pub.publish(grab_msg)
        
        if self.packet_count % 60 == 0:
            self.get_logger().info(
                f"{hand_type.title()} hand: pos=({position[0]:.1f}, {position[1]:.1f}, {position[2]:.1f}) "
                f"grab={hand.get('grab_strength', 0.0):.2f}"
            )
    
    def print_stats(self):
        """Print performance statistics"""
        elapsed = time.time() - self.start_time
        if elapsed > 0:
            avg_rate = self.packet_count / elapsed
            time_since_last = time.time() - self.last_packet_time if self.last_packet_time > 0 else 0
            
            self.get_logger().info(
                f"Stats: {self.packet_count} packets, {avg_rate:.1f} pkt/s, "
                f"last: {time_since_last:.1f}s ago"
            )
    
    def destroy_node(self):
        """Clean shutdown"""
        self.running = False
        if self.socket:
            self.socket.close()
        super().destroy_node()

def main(args=None):
    print("""
╔══════════════════════════════════════════════════════════════╗
║                ROS2 HAND TRACKING RECEIVER                   ║
║             Receiving Leap Motion data over UDP             ║
╚══════════════════════════════════════════════════════════════╝
""")
    
    rclpy.init(args=args)
    
    try:
        node = HandReceiver()
        
        print("🔄 Node running... Press Ctrl+C to stop")
        print("📡 Waiting for hand tracking data from Windows...")
        print("📊 Published topics:")
        print("   - /hand_tracking/left_palm")
        print("   - /hand_tracking/right_palm") 
        print("   - /hand_tracking/left_grab_strength")
        print("   - /hand_tracking/right_grab_strength")
        print()
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\nℹ️ Shutting down...")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()