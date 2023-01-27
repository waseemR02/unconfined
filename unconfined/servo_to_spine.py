import json
# from socket import socket
import socket
from numpy import interp 

import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8MultiArray as unconfined_msgs

SOCKET_ADDR = "/tmp/hardware.sock"


class ServoToSpine(Node):
    def __init__(self) -> None:
        super().__init__('servo_to_spine')

        self.hardware_socket = self.prepare_socket(addr=SOCKET_ADDR)

        self.subscription = self.create_subscription(
            unconfined_msgs, 'servo', self.servo_write,
            10
        )
        self.subscription

    def prepare_socket(self, addr):     
        """Connect to a UNIX socket on addr and return the socket object."""
        sock =  socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.connect(addr)
        return sock

    def servo_write(self, angles):
        """Write pan and tilt angles in microseconds to spine in json"""
        pan_degrees = angles.data[1]
        tilt_degrees = angles.data[0]

        # interpolates [0,180] angles --> [500,1500] microseconds
        pan_microseconds = interp(pan_degrees, [0,180], [500,2500])
        tilt_microseconds = interp(tilt_degrees, [0,180], [500,2500])

        self.hardware_socket.sendall(json.dumps({"ServoWrite": {"servo": "digital_camera1_pan", "position": int(pan_microseconds)}}).encode("utf8"))
        self.hardware_socket.sendall(json.dumps({"ServoWrite": {"servo": "digital_camera1_tilt", "position": int(tilt_microseconds)}}).encode("utf8"))

def main(args=None):
    rclpy.init(args=args)

    servo_to_spine_node = ServoToSpine()

    rclpy.spin(servo_to_spine_node)

    servo_to_spine_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
