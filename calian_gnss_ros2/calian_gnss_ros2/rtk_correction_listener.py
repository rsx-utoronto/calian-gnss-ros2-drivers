import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from calian_gnss_ros2_msg.msg import CorrectionMessage


class RTKCorrectionListener(Node):
    def __init__(self):
        super().__init__("rtk_correction_listener")

        self.declare_parameter("host", "192.168.2.88")
        self.declare_parameter("port", 5409)

        self.host = self.get_parameter("host").get_parameter_value().string_value
        self.port = self.get_parameter("port").get_parameter_value().integer_value

        self.pub = self.create_publisher(CorrectionMessage, "rtk_corrections", 10)

        self.get_logger().info(
            f"RTK Correction Listener starting on {self.host}:{self.port}"
        )
        self.socket_listener()

    def socket_listener(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.host, self.port))
            s.listen()

            self.get_logger().info("Ready for connection from Windows")

            while rclpy.ok():
                conn, addr = s.accept()
                with conn:
                    self.get_logger().info(f"Connected by {addr}")

                    while rclpy.ok():
                        correction = CorrectionMessage()
                        correction.header = Header()
                        correction.header.stamp = self.get_clock().now().to_msg()
                        correction.header.frame_id = "gps"

                        data = conn.recv(1024)
                        if data:
                            correction.message = list(data)
                            self.pub.publish(correction)
                        else:
                            break

                self.get_logger().info(
                    "Shutting down TruPrecision ROS listener node."
                )


def main(args=None):
    rclpy.init(args=args)
    node = RTKCorrectionListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
