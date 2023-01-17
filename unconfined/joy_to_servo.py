"""ROS2 Node for publishing teleop commands from joystick to a topic /servo"""
import cv2
import zmq
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8MultiArray as unconfined_msgs

from unconfined.stitch_image import create_panorama

camera = cv2.VideoCapture(2)

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
        
        # Creating two callback groups for asynchronous callback and independent flow
        # callback_group_a includes update_frames_callback() and servo_angle_publisher() and cmd_updater()
        # callback group_b includes the panorama_callback() 

        callback_group_a = MutuallyExclusiveCallbackGroup()
        callback_group_b = MutuallyExclusiveCallbackGroup()
        
        
        self.subscription = self.create_subscription(
                Joy, "joy", self.cmd_updater, qos_profile=10)
        self.subscription
        
        # Create publisher for publishing servo angles to /servo
        self.servo_angle_publisher = self.create_publisher(unconfined_msgs, "servo", 10)
        
        # Callback time for servo_angle_callback which updates servo angles 
        servo_angle_publisher_callback_time = 0.1
        self.timer = self.create_timer(servo_angle_publisher_callback_time, 
                                       self.servo_angle_callback, callback_group=callback_group_a)

        # stream portion
        context = zmq.Context()
        self.footage_socket = context.socket(zmq.PUB)
        self.footage_socket.connect('tcp://192.168.0.106:5555')
        
        # update frames every 0.01 seconds
        self.timer = self.create_timer(0.01,self.update_frames_callback, callback_group=callback_group_a)

        # panorama
        self.take_panorama = False
        self.called_panorama_shot_caller = False
        self.increment = 1
        self.current_taking_panorama = False
        self.error_panorama = False

        # image list for stitching them later
        self.images = []

        # Timer for calling panorama_callback()
        self.timer = self.create_timer(0.1, self.panorama_callback, callback_group=callback_group_b)

    def update_frames_callback(self):
        """
        Read frames from camera and update self.frames every 0.1 seconds
        """
        status, self.frame = camera.read()
        encoded, buffer = cv2.imencode('.jpg', self.frame)
        self.footage_socket.send(buffer)
    
    def panorama_callback(self):
        """
        Check if take_panorama is true every 0.1 seconds and call panorama_mode() when true

        Runs in a separate thread so it should block the program
        """
        if self.take_panorama:
            if not self.current_taking_panorama:
                self.current_taking_panorama = True
                self.SERVO[1] = 20
            self.panorama_mode()

        if self.error_panorama:
            self.take_panorama = True
            self.get_logger().info(f"Error while stitching!! Trying again")
            self.error_panorama = False
            

    def cmd_updater(self,joy):
        """Calls teleop_cmd_caller() and panorama shot according to joy commands"""
        if joy.buttons[10] == 1:
            self.take_panorama = True           
        if not self.current_taking_panorama:
            self.teleop_cmd_caller(joy)

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

        return list(map(self._set_under_range, required_servo_state))
        

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


    def panorama_mode(self):
        """
        Collect frames when called and append to self.images and stop when servo_angle is over 120 degrees
        """
        self.images.append(self.frame)
        self.SERVO[1] += self.increment

        if not self.SERVO[1] < 120:
            self.take_panorama = False
            self.current_taking_panorama = False
            self.get_logger().info("Started stitching images together")
            self.error_panorama = create_panorama(self.images,f"{datetime.now()}")
            self.get_logger().info("Finished stitching images")
            self.images = []


def main(args=None):
    """Driver code"""
    rclpy.init(args = args)

    joy_to_servo_node = JoyToServo()

    # rclpy.spin(joy_to_servo_node)

    executor = MultiThreadedExecutor()
    executor.add_node(joy_to_servo_node)

    executor.spin()
    
    camera.release()
    cv2.destroyAllWindows()

    joy_to_servo_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
