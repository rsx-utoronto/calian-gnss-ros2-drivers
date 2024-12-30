#!/usr/bin/env python3


"""
Filename: rtk_correction_listner.py

Listen to TruPrecision serial data sent from Windows computer on local network over socket and publish ROS topic

Server-side (Ubuntu, ROS) script

Client-side code script is available at
https://github.com/rsx-utoronto/calian-windows-transfer/blob/main/reader-truprecision.py

Robotics for Space Exploration - University of Toronto

Author: Jason Li <jasonli.li@mail.utoronto.ca>
Date: 2024-11-23
"""





import socket
import rospy
from calian_gnss_ros2_msg.msg import CorrectionMessage


class RTKListener:
    def __init__(self, host, port):
        # ROS setup -------------------------------------

        rospy.init_node('rtk_listener', anonymous=True)
        self.topic_name = 'rtk_corrections'
        self.pub = rospy.Publisher(self.topic_name, CorrectionMessage, queue_size=10)

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

            rospy.loginfo("Ready for connection from Windows")

            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                # Found connection
                conn, addr = s.accept()
                with conn:
                    rospy.loginfo(f"Connected by {addr}")

                    while not rospy.is_shutdown():
                        Correction = CorrectionMessage()
                        Correction.header.stamp = rospy.Time.now()
                        Correction.header.frame_id = ""

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

                rospy.loginfo("Shutting down TruPrecision ROS listener node.")
                rate.sleep()

if __name__ == '__main__':
    # Ensure that these are the same on the Windows-side Python script.
    # HOST should be the Ubuntu computer's local IP, and
    # PORT should be an available port on the Ubuntu computer.
    # This is to send the data to the Ubuntu computer.
    HOST = "192.168.2.88"
    PORT = 5409

    rtk_listener = RTKListener(HOST, PORT)
    rtk_listener.socket_listener()