# joy_udp_sender.py
# Copyright 2025
# Licensed under the Apache License, Version 2.0

import json
import socket
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

DEFAULT_TARGET_IP = '255.255.255.255'
DEFAULT_PORT = 5005

A_INDEX = 0  # BTN_SOUTH (A button)
Y_INDEX = 3  # BTN_NORTH (Y button)

class JoyUdpSender(Node):
    def __init__(self):
        super().__init__('joy_udp_sender')
        
        self.declare_parameter('target_ip', DEFAULT_TARGET_IP)
        self.declare_parameter('port', DEFAULT_PORT)
        
        self.target_ip = self.get_parameter('target_ip').value
        self.port = self.get_parameter('port').value
        
        # Joy subscription
        self.sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        
        # Local IP
        self.local_ip = self.get_local_ip()
        
        self.seq = 0
        self.packet_count = 0
        
        # Previous button states
        self.prev_a = 0
        self.prev_y = 0
        
        self.get_logger().info('='*60)
        self.get_logger().info('Joystick UDP Sender (Edge Detection)')
        self.get_logger().info(f'My IP: {self.local_ip}')
        self.get_logger().info(f'Broadcasting to: {self.target_ip}:{self.port}')
        self.get_logger().info('Sends A and Y button states')
        self.get_logger().info('='*60)
        
        self.heartbeat_timer = self.create_timer(10.0, self.heartbeat)

    def get_local_ip(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(('8.8.8.8', 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return socket.gethostname()

    def heartbeat(self):
        self.get_logger().info(
            f'Heartbeat: Sent {self.packet_count} button changes from {self.local_ip}'
        )

    def joy_callback(self, joy_msg: Joy):
        try:
            a_val = int(joy_msg.buttons[A_INDEX]) if len(joy_msg.buttons) > A_INDEX else 0
            y_val = int(joy_msg.buttons[Y_INDEX]) if len(joy_msg.buttons) > Y_INDEX else 0
            
            # Return if no state change
            if a_val == self.prev_a and y_val == self.prev_y:
                return
            
            # Send UDP when state changes
            payload = {
                "seq": self.seq,
                "sender_ip": self.local_ip,
                "buttons": {"A": a_val, "Y": y_val}
            }
            
            data = json.dumps(payload).encode('utf-8')
            self.sock.sendto(data, (self.target_ip, self.port))
            self.packet_count += 1
            
            # Log state changes
            if a_val != self.prev_a:
                if a_val == 1:
                    self.get_logger().info(f'SEND seq={self.seq} | A PRESSED')
                else:
                    self.get_logger().info(f'SEND seq={self.seq} | A RELEASED')
            
            if y_val != self.prev_y:
                if y_val == 1:
                    self.get_logger().info(f'SEND seq={self.seq} | Y PRESSED')
                else:
                    self.get_logger().info(f'SEND seq={self.seq} | Y RELEASED')
            
            # Update previous states
            self.prev_a = a_val
            self.prev_y = y_val
            
            self.seq += 1
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def destroy_node(self):
        self.get_logger().info(f'Total: {self.packet_count} button changes sent')
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoyUdpSender()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()