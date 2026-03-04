# joy_udp_receiver.py
# Copyright 2025
# Licensed under the Apache License, Version 2.0

import json
import socket
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8

DEFAULT_PORT = 5005
MAX_ROBOTS = 7
TIMEOUT_SEC = 10.0

class JoyUdpReceiver(Node):
    def __init__(self):
        super().__init__('joy_udp_receiver')
        
        self.declare_parameter('port', DEFAULT_PORT)
        self.port = self.get_parameter('port').value
        
        # /edie8/station/out publisher
        self.pub_station_out = self.create_publisher(UInt8, '/edie8/station/out', 10)
        
        # Local IP
        self.local_ip = self.get_local_ip()
        
        # UDP socket (broadcast receive)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(('0.0.0.0', self.port))
        self.sock.settimeout(0.0)
        
        # Connected senders tracking
        self.senders = {}
        self.total_packets = 0
        
        # Timers
        self.timer = self.create_timer(0.001, self.poll_udp)
        self.heartbeat_timer = self.create_timer(5.0, self.heartbeat)
        self.cleanup_timer = self.create_timer(2.0, self.cleanup_stale)
        
        self.get_logger().info('='*60)
        self.get_logger().info('Joystick UDP Receiver')
        self.get_logger().info(f'My IP: {self.local_ip}')
        self.get_logger().info(f'Listening on port: {self.port}')
        self.get_logger().info(f'Max senders: {MAX_ROBOTS}')
        self.get_logger().info('A button -> publishes 3 to /edie8/station/out')
        self.get_logger().info('Y button -> publishes 4 to /edie8/station/out')
        self.get_logger().info('='*60)

    def get_local_ip(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(('8.8.8.8', 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return socket.gethostname()

    def cleanup_stale(self):
        """Remove timed-out senders"""
        current_time = time.time()
        to_remove = []
        
        for sender_ip, info in self.senders.items():
            if current_time - info['last_time'] > TIMEOUT_SEC:
                to_remove.append(sender_ip)
        
        for sender_ip in to_remove:
            self.get_logger().warn(
                f'DISCONNECTED: {sender_ip} (timeout {TIMEOUT_SEC}s)'
            )
            del self.senders[sender_ip]
            self.log_connection_status()

    def heartbeat(self):
        active_count = len(self.senders)
        if active_count > 0:
            sender_list = ', '.join(self.senders.keys())
            self.get_logger().info(
                f'Heartbeat [{self.local_ip}] Connected: {active_count}/{MAX_ROBOTS} | '
                f'Packets: {self.total_packets}'
            )
            self.get_logger().info(f'IPs: {sender_list}')
        else:
            self.get_logger().warn(
                f'[{self.local_ip}] No sender connected (0/{MAX_ROBOTS})'
            )

    def log_connection_status(self):
        active = len(self.senders)
        self.get_logger().info(
            f'Status: {active}/{MAX_ROBOTS} senders active'
        )

    def poll_udp(self):
        try:
            while True:
                data, addr = self.sock.recvfrom(2048)
                self.handle_packet(data, addr)
        except BlockingIOError:
            pass
        except Exception as e:
            self.get_logger().warn(f'Recv error: {e}')

    def handle_packet(self, data: bytes, addr):
        try:
            obj = json.loads(data.decode('utf-8'))
            
            seq = obj.get('seq', -1)
            sender_ip = obj.get('sender_ip', addr[0])
            a = bool(obj.get('buttons', {}).get('A', 0))
            y = bool(obj.get('buttons', {}).get('Y', 0))
            
            current_time = time.time()
            
            # Detect new sender
            if sender_ip not in self.senders:
                if len(self.senders) >= MAX_ROBOTS:
                    self.get_logger().warn(
                        f'Max senders reached ({MAX_ROBOTS}). Ignoring {sender_ip}'
                    )
                    return
                
                self.senders[sender_ip] = {
                    'last_time': current_time,
                    'count': 0
                }
                
                self.get_logger().info('='*60)
                self.get_logger().info(
                    f'NEW SENDER CONNECTED: {sender_ip}'
                )
                self.get_logger().info(
                    f'Total: {len(self.senders)}/{MAX_ROBOTS} senders'
                )
                self.get_logger().info('='*60)
                self.log_connection_status()
            
            # Update sender info
            self.senders[sender_ip]['last_time'] = current_time
            self.senders[sender_ip]['count'] += 1
            self.total_packets += 1
            
            # A button pressed -> publish 3
            if a:
                msg = UInt8()
                msg.data = 3
                self.pub_station_out.publish(msg)
                self.get_logger().info(
                    f'A [{sender_ip}] seq={seq} | A PRESSED -> Published 3'
                )
            
            # Y button pressed -> publish 4
            if y:
                msg = UInt8()
                msg.data = 4
                self.pub_station_out.publish(msg)
                self.get_logger().info(
                    f'Y [{sender_ip}] seq={seq} | Y PRESSED -> Published 4'
                )
            
        except Exception as e:
            self.get_logger().warn(f'Packet error: {e}')

    def destroy_node(self):
        self.get_logger().info(f'Total: {self.total_packets} packets')
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoyUdpReceiver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()