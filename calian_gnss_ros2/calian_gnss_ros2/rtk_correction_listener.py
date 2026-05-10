#!/usr/bin/env python3


"""
Filename: rtk_correction_listener.py

Listen to TruPrecision serial data sent from Windows computer on local network over socket and publish ROS topic

Server-side (Ubuntu, ROS) script

Client-side code script is available at
https://github.com/rsx-utoronto/calian-windows-transfer/blob/main/reader-truprecision.py

Robotics for Space Exploration - University of Toronto

Author: Jason Li <jasonli.li@mail.utoronto.ca>
Date: 2024-11-23
"""


import socket
import rclpy
from rclpy.node import Node
from calian_gnss_ros2_msg.msg import CorrectionMessage


class RTKListener(Node):
    def __init__(self, host, port):
        # ROS setup -------------------------------------
        super().__init__('rtk_listener')
        
        self.topic_name = 'rtk_corrections'
        self.pub = self.create_publisher(CorrectionMessage, self.topic_name, 10)

        # -----------------------------------------------

        # Ensure that these are the same on the Windows-side Python script.
        # HOST should be the Ubuntu computer's local IP, and
        # PORT should be an available port on the Ubuntu computer.
        # This is to send the data to the Ubuntu computer.

        self.HOST = host
        self.PORT = port
    
    def socket_listener(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.HOST, self.PORT))
            s.listen()

            self.get_logger().info("Ready for connection from Windows")

            rate = self.create_rate(10)
            while rclpy.ok():
                # Found connection
                conn, addr = s.accept()
                with conn:
                    self.get_logger().info(f"Connected by {addr}")

                    while rclpy.ok():
                        Correction = CorrectionMessage()
                        Correction.header.stamp = self.get_clock().now().to_msg()
                        Correction.header.frame_id = "gps"

                        # Received data
                        data = conn.recv(1024)
                        if data:

                            # byte_array = UInt8MultiArray()
                            byte_array = list(data)
                            Correction.message = byte_array
                            self.pub.publish(Correction)                                

                        # Disconnect when data has stopped
                        # Outer while loop will wait for connection again
                        else:
                            break
                        rate.sleep()

                self.get_logger().info("Shutting down TruPrecision ROS listener node.")
                rate.sleep()

def main(args=None):
    # Ensure that these are the same on the Windows-side Python script.
    # HOST should be the Ubuntu computer's local IP, and
    # PORT should be an available port on the Ubuntu computer.
    # This is to send the data to the Ubuntu computer.
    HOST = "192.168.0.125"
    PORT = 5409

    rclpy.init(args=args)
    
    rtk_listener = RTKListener(HOST, PORT)
    
    try:
        rtk_listener.socket_listener()
    except KeyboardInterrupt:
        pass
    finally:
        rtk_listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
