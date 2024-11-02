from calian_gnss_ros2.serial_module import SerialUtilities
import rospy
import serial
import concurrent.futures
from serial.tools.list_ports import comports
from calian_gnss_ros2.logging import Logger, LoggingLevel, SimplifiedLogger  # Adjust import for ROS 1

class UniqueIdFinder:
    def __init__(self):
        rospy.init_node("unique_id_finder")
        self.save_logs = rospy.get_param("~save_logs", False)
        self.log_level = rospy.get_param("~log_level", LoggingLevel.Info)

        internal_logger = Logger(rospy.loginfo)
        internal_logger.toggle_logs(self.save_logs)
        internal_logger.setLevel(self.log_level)

        self.logger = SimplifiedLogger("unique_id_finder")
        self.logger.info("Processing connected ports.....")

        ports = comports()
        if len(ports) == 0:
            self.logger.warn("No ports connected.")
        else:
            with concurrent.futures.ThreadPoolExecutor() as executor:
                for port in ports:
                    if "Standard" in port.description:
                        try:
                            self.logger.debug("Connecting to port " + port.device)
                            standard_port = serial.Serial(port.device, 230400)
                            thread = executor.submit(
                                SerialUtilities.extract_unique_id_of_port,
                                standard_port=standard_port,
                                timeout=3,
                            )
                            unique_id_of_port = thread.result(3)
                            self.logger.info(
                                port.device + " : " + unique_id_of_port.upper()
                            )
                        except Exception as e:
                            self.logger.error(
                                "Cannot get unique id of the port " + port.device + ": " + str(e)
                            )
                        finally:
                            standard_port.close()
                            standard_port = None
                            self.logger.debug("Moving on to next port.")

        self.logger.info("All ports are processed.")

if __name__ == "__main__":
    unique_id_finder = UniqueIdFinder()
