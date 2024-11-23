#!/usr/bin/python3

import base64
import rospy
from std_msgs.msg import Header
from ably import AblyRealtime
from ably.types.message import Message
from ably.types.connectionstate import (
    ConnectionEvent,
    ConnectionState,
    ConnectionStateChange,
)

from pyrtcm import RTCMReader

from calian_gnss_ros2_msg.msg import CorrectionMessage
from calian_gnss_ros2.logging import Logger, LoggingLevel, SimplifiedLogger


class RemoteRtcmCorrectionsHandler:
    def __init__(self):
        rospy.init_node("remote_rtcm_corrections_handler", anonymous=True)

        # region Parameters declaration
        self.key = rospy.get_param("~key", "")
        self.channel = rospy.get_param("~channel", "")
        self.save_logs = rospy.get_param("~save_logs", False)
        self.log_level = LoggingLevel(rospy.get_param("~log_level", LoggingLevel.Info))
        # endregion

        internal_logger = Logger(rospy.log)
        internal_logger.toggle_logs(self.save_logs)
        internal_logger.setLevel(self.log_level)
        self.logger = SimplifiedLogger("remote_rtcm_corrections_handler")

        # Publisher to publish RTCM corrections to Rover
        self.rtcm_publisher = rospy.Publisher("rtcm_corrections", CorrectionMessage, queue_size=50)

        self.ably = AblyRealtime(self.key, auto_connect=False)
        self.ably.connect()
        self.ably.connection.on(self.log_events)

        self.__process()

    def __process(self):
        self.logger.info("Connecting to real-time...")
        self.__channel = self.ably.channels.get(self.channel)
        self.__channel.subscribe("RTCM Corrections", self.__process_incoming_messages)

        rate = rospy.Rate(15)  # 15 Hz
        while not rospy.is_shutdown():
            self.ably.connection.ping()  # Implement your own ping method if necessary
            self.__channel.publish_message(
                Message(
                    name="Rover Ping",
                    data=self.ably.connection.connection_manager.connection_id,
                )
            )
            rate.sleep()
        self.ably.connection.once(ConnectionState.CLOSED)

    def log_events(self, args: ConnectionStateChange):
        self.logger.info("Connection state changed from {} to {}".format(args.previous, args.current))

    def __process_incoming_messages(self, message: Message):
        try:
            for msg in message.data:
                # Use the RTCMReader to process the decoded RTCM message
                parsed_data = RTCMReader.parse(base64.b64decode(msg))
                
                rtcmMessage = CorrectionMessage(
                    header=Header(
                        stamp=rospy.Time.now(),
                        frame_id=self._frame_id,
                    ),
                    message=parsed_data,  # Assuming parsed_data contains relevant RTCM data
                )
                self.rtcm_publisher.publish(rtcmMessage)
        except Exception as e:
            self.logger.error("Exception while processing received message: {}".format(e))


def main():
    rtcm_data_handler = RemoteRtcmCorrectionsHandler()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

    # In ROS 1, nodes are cleaned up automatically on shutdown,
    # but you can add explicit cleanup here if necessary.

if __name__ == "__main__":
    main()
