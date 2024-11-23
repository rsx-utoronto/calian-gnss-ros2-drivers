#!/usr/bin/python3

from typing import Literal
import rospy
import sys
import base64
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
from calian_gnss_ros2.pointperfect_module import PointPerfectModule
from calian_gnss_ros2.serial_module import UbloxSerial
from pynmeagps import NMEAMessage
from pyrtcm import RTCMReader
from calian_gnss_ros2_msg.msg import GnssSignalStatus, CorrectionMessage
from calian_gnss_ros2.logging import Logger, LoggingLevel, SimplifiedLogger
from std_srvs.srv import Empty
from nmea_msgs.msg import Sentence


class Gps:
    def __init__(self, mode: Literal["Disabled", "Heading_Base", "Rover"] = "Disabled") -> None:
        rospy.init_node("calian_gnss_gps")

        internal_logger = Logger(rospy.loginfo)
        # Parameters initialization
        self.unique_id = rospy.get_param("~unique_id", "")
        self.baud_rate = rospy.get_param("~baud_rate", 230400)
        self.use_corrections = rospy.get_param("~use_corrections", True)
        self.corrections_source = rospy.get_param("~corrections_source", "PointPerfect")
        self.save_logs = rospy.get_param("~save_logs", False)
        self.log_level = rospy.get_param("~log_level", LoggingLevel.Info)
        self._frame_id = rospy.get_param("~frame_id", "gps")

        internal_logger.toggle_logs(self.save_logs)
        internal_logger.setLevel(self.log_level)
        self.logger = SimplifiedLogger(mode + "_GPS")
        self.ser = UbloxSerial(
            self.unique_id,
            self.baud_rate,
            mode,
            self.use_corrections,
            self.corrections_source,
        )

        # ROS Publishers and Subscribers
        if mode == "Heading_Base":
            self.rtcm_publisher = rospy.Publisher("rtcm_corrections", CorrectionMessage, queue_size=100)
            self.base_status_publisher = rospy.Publisher("base_gps_extended", GnssSignalStatus, queue_size=50)
            self.ser.rtcm_message_found += self.handle_rtcm_message
            self.rtcm_msg_pool = []
            rospy.Timer(rospy.Duration(0.5), self.publish_pooled_rtcm)
        elif mode == "Rover":
            self.rtcm_subscriber = rospy.Subscriber("rtcm_corrections", CorrectionMessage, self.handle_rtcm_message)
            self.gps_publisher = rospy.Publisher("gps", NavSatFix, queue_size=50)
            self.gps_status_publisher = rospy.Publisher("gps_extended", GnssSignalStatus, queue_size=50)
        elif mode == "Disabled":
            self.gps_publisher = rospy.Publisher("gps", NavSatFix, queue_size=50)
            self.gps_status_publisher = rospy.Publisher("gps_extended", GnssSignalStatus, queue_size=50)

        rospy.Timer(rospy.Duration(1), self.get_status)

        if self.use_corrections:
            self.correction_subscriber = rospy.Subscriber("corrections", CorrectionMessage, self.handle_correction_message)
            if self.corrections_source == "PointPerfect":
                self._pp_client = rospy.ServiceProxy("restart", Empty)
                rospy.Timer(rospy.Duration(5), self.__reconnect_pointperfect_if_needed)
            else:
                self.nmea_publisher = rospy.Publisher("nmea", Sentence, queue_size=100)
                rospy.Timer(rospy.Duration(1), self.send_nmea_message)
                self._recent_nmea_gga = ""
                self.ser.nmea_message_found += self.handle_nmea_message

    def handle_correction_message(self, message) -> None:
        self.logger.debug("Sending correction message: " + message.message.tobytes().hex(" "))
        self.ser.send(message.message.tobytes())

    def handle_nmea_message(self, nmeaMessage: NMEAMessage) -> None:
        if nmeaMessage.identity == "GNGGA" and self.use_corrections and self.corrections_source == "Ntrip":
            self._recent_nmea_gga = nmeaMessage.serialize().decode("utf-8")

    def handle_rtcm_message(self, rtcmMessage) -> None:
        if mode == "Heading_Base":
            msg = CorrectionMessage(
                header=Header(stamp=rospy.Time.now(), frame_id=self._frame_id),
                message=rtcmMessage.serialize(),
            )
            self.rtcm_msg_pool.append(msg)
        else:
            rmg = RTCMReader.parse(rtcmMessage.message)
            self.ser.send(rmg.serialize())
            self.logger.debug("Received RTCM message with identity: " + rmg.identity)

    def get_status(self) -> None:
        status = self.ser.get_status()

        header = Header(stamp=rospy.Time.now(), frame_id=self._frame_id)
        status.header = header

        if not status.valid_fix:
            return

        if mode == "Rover" or mode == "Disabled":
            msg = NavSatFix()
            msg.header = header
            msg.latitude = status.latitude
            msg.longitude = status.longitude
            msg.altitude = status.altitude
            msg.position_covariance = status.position_covariance
            msg.position_covariance_type = status.position_covariance_type
            msg.status = status.status
            self.gps_publisher.publish(msg)
            self.gps_status_publisher.publish(status)
            self.logger.debug(
                "Published GPS data - Latitude: {:.6f}, Longitude: {:.6f}".format(status.latitude, status.longitude)
            )
        else:
            self.base_status_publisher.publish(status)

    def send_nmea_message(self) -> None:
        self.nmea_publisher.publish(
            Sentence(
                header=Header(stamp=rospy.Time.now(), frame_id=self._frame_id),
                sentence=self._recent_nmea_gga,
            )
        )

    def publish_pooled_rtcm(self, event) -> None:
        pooled_msgs = list(self.rtcm_msg_pool)
        self.rtcm_msg_pool.clear()
        for msg in pooled_msgs:
            self.rtcm_publisher.publish(msg)
            self.logger.debug("Published RTCM message -> " + str(msg.message))

    def __reconnect_pointperfect_if_needed(self, event) -> None:
        if self.use_corrections:
            sptn_key = self.ser.poll_once("RXM", "RXM-SPARTN-KEY")
            if sptn_key and sptn_key.numKeys == 0 and self._pp_client.wait_for_service(timeout=1.0):
                self._pp_client.call(Empty.Request())


def main():
    args = rospy.myargv(argv=sys.argv)
    gps = Gps(mode=args[1] if len(args) > 1 else "Disabled")
    rospy.spin()


if __name__ == "__main__":
    main()
