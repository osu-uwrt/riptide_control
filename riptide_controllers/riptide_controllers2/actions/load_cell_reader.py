import sys
import rclpy
import serial

from std_msgs.msg import Int32
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default


class SerialDecoder:
    def __init__(self):
        self.reset_state()

    def reset_state(self):
        self._state = 0
        self._idx = 0
        self._valid = False

    def process_byte(self, val: bytes):
        assert len(val) == 1, "Byte must be single length"
        val = val[0]
        # Handle start char
        if val & 0x80:
            self.reset_state()
            val &= 0x7F
        elif self._idx == 0:
            # Don't process packets if we need a start character
            print("Waiting for sync...")
            return

        # Don't process any more bytes after 4
        if self._idx > 3:
            print("Garbage data received after packet, waiting for sync")
            return
        elif self._idx == 3:
            # If third byte, handle checksum special
            checksum = val >> 3
            val &= 0x7
            self._state |= val << (8 * self._idx)
            self._idx += 1

            computed_checksum = 0b1011
            tmp_state = self._state
            for _ in range(24//4):
                computed_checksum ^= tmp_state & 0xF
                tmp_state >>= 4

            self._valid = (computed_checksum == checksum)
            if not self._valid:
                print("Failed to decode, invalid packet ({} vs {})".format(
                    hex(computed_checksum), hex(checksum)))
                self.reset_state()
        else:
            self._state |= val << (8 * self._idx)
            self._idx += 1

    def available(self) -> bool:
        return self._valid

    def get_value(self) -> bool:
        if not self._valid:
            raise RuntimeError("No valid reading available")

        if self._state < 0x80_0000:
            return self._state
        else:
            # Perform twos-complement on result
            return -1 * (0x100_0000 - self._state)


def main(args=None):
    rclpy.init(args=args)
    node = Node('thruster_force_gauge_publisher')
    node.declare_parameter("port", "/dev/ttyACM0")
    pub = node.create_publisher(
        Int32, "force_gauge/force", qos_profile_system_default)

    openedPort = False
    while(openedPort == False):
        try:
            ser = serial.Serial(node.get_parameter("port").value, 115200)
            decoder = SerialDecoder()
            openedPort = True
        except:
            node.get_logger().info("Cannot open port: " + str(node.get_parameter("port").value))


    while True:
        while ser.in_waiting:
            decoder.process_byte(ser.read(1))
            if decoder.available():
                pub.publish(Int32(data=decoder.get_value()))
        # If you change the publish rates on the arduino, adjust this
        rclpy.spin_once(node, timeout_sec=0.005)

    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
