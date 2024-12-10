import threading

import serial

import rclpy
from rclpy.node import Node
from nmea_msgs.msg import Sentence


class NmeaReader(Node):
    def __init__(self):
        super().__init__("nmea_node")
        serial_port = self.declare_parameter("port", "/dev/ttyUSB0").value
        serial_baud = self.declare_parameter("baud", 9600).value
        frame_id = self.declare_parameter("frame_id", "gps").value

        self.frame_id = frame_id
        self.publisher = self.create_publisher(Sentence, "nmea_sentence", 10)
        self.gps = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=2)

        thread = threading.Thread(target=self.looper)
        thread.start()

    def looper(self):
        try:
            while rclpy.ok():
                data = self.gps.readline().strip().decode("ascii")

                sentence = Sentence()
                sentence.header.stamp = self.get_clock().now().to_msg()
                sentence.header.frame_id = self.frame_id
                sentence.sentence = data
                self.publisher.publish(sentence)
        except KeyboardInterrupt:
            pass


def main(args=None):
    rclpy.init(args=args)

    node = NmeaReader()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
