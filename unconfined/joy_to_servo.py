"""ROS2 Node for publishing teleop commands from joystick to a topic /servo"""
import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msgs import Joy
from std_msgs.msg import UInt8MultiArray as unconfined_msgs

class JoyToServo(Node):
    """
    JoyToServo Node class
    Initialises zmq socket, VideoCapture and the node which then 
    send images over the network and publish appropriate commands 
    to the /servo topic 
    """
    def __init__(self):
        """Initialises the node"""
        super().__init__("joy_to_servo")
        
        # default values for the servo_angle
        self.SERVO = [90,90]
        self.subscription = self.create_subscription(
                Joy, "joy", self.teleop_cmd_caller, 10)
        self.subscription

        self.servo_angle_publisher = self.create_publisher(servo_angle, "servo", 10)
        
        servo_angle_publisher_callback_time = 0.1
        self.timer = self.create_timer(servo_angle_publisher_callback_time, 
                                       self.servo_angle_callback)

    
    def teleop_cmd_caller(self, joy):
        """Calls the teleop_cmd_map function and updates self.SERVO"""
    
        self.SERVO = self.teleop_cmd_map(joy, 2, self.SERVO)
    

    def teleop_cmd_map(self, joy, increment, current_servo_state):
        """Simple Mapping of joy node's teleop commands to 
        appropriate servo angles
        """

        tilt_axis = joy.axes[5]
        pan_axis = joy.axes[4]

        required_servo_state = current_servo_state

        if tilt_axis == 1:
            required_servo_state[0] = current_servo_state[0] + increment
        elif tilt_axis == -1:
            required_servo_state[0] = current_servo_state[0] - increment

        if pan_axis == -1:
            required_servo_state[1] = current_servo_state[1] + increment
        elif pan_axis == 1:
            required_servo_state[1] = current_servo_state[1] - increment

        return map(self._set_under_range, required_servo_state)
        

    def _set_under_range(self, angle):
        """Return the passed angle under the range of (0,180)"""
        if angle < 0:
            return 0
        elif angle > 180:
            return 180

        return angle


    def servo_angle_callback(self):
        """Publish the required servo angles to /servo"""

        self.get_logger().info(f"Publishing: Pan:{self.SERVO[1]}, Twist:{self.SERVO[0]}")
        msg = unconfined_msgs()
        msg.data = self.SERVO
        self.servo_angle_publisher.publish(msg)


def main(args=None):
    """Driver code"""
    rclpy.init(args = args)

    joy_to_drive_node = JoyToServo()

    rclpy.spin(joy_to_drive_node)

    joy_to_drive_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
