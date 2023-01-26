"""ROS2 Node for viewing the stream from the joy_to_servo node or any zmq socket"""
import cv2
import zmq
import numpy as np
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from cv_bridge import CvBridge

from sensor_msgs.msg import Joy, Image
from std_msgs.msg import UInt8MultiArray as unconfined_msgs

from unconfined.stitch_image import PanoramaMaker

class ViewStream(Node):
    """
    ViewStream class
    node initialises the zmq socket and binds to it to
    receive images from the server while also simultaneously 
    commands the joy_to_servo to take panorama
    """
    def __init__(self):
        """Initialises the node"""
        super().__init__("view_stream")

        # create a separate callback group for the stream
        callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()


        # Create a subscription to the joy topic and the servo topic to cooridinate the panorama
        self.subscription = self.create_subscription(
                Joy, "joy", self.panorama_check_callback, 10, callback_group=callback_group)
        self.subscription = self.create_subscription(
                unconfined_msgs, "servo", self.panorama_mode, 10)
        self.subscription


        # Create a image publisher and a CvBridge object to convert the frames to ROS2 messages
        # and publish them to stream
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, "stream", 10)


        # Create parmaeter list for socket_ip and socket port and panorama_mode
        self.declare_parameter("socket_ip", "192.168.1.99")
        self.declare_parameter("socket_port", 5555)
        self.declare_parameter("panorama_mode", False)

        # Get the parameters for socket_ip and socket_port
        self.socket_ip = self.get_parameter("socket_ip").value
        self.socket_port = self.get_parameter("socket_port").value
        self.panorama_mode = self.get_parameter("panorama_mode").value

        # Print the params once the node starts
        self.get_logger().info("Socket set to %s:%s" % (self.socket_ip, self.socket_port))
        self.get_logger().info("Panorama Mode set to : %s" % self.panorama_mode)

        # Create a zmq context
        context = zmq.Context()
        self.footage_socket = context.socket(zmq.SUB)
        self.footage_socket.bind('tcp://%s:%s' % (self.socket_ip, self.socket_port))
        self.footage_socket.setsockopt_string(zmq.SUBSCRIBE, np.unicode(''))

        # Create a timer to get the frames from the server and display them
        self.timer = self.create_timer(0.01, self.get_frames_callback, callback_group=callback_group)

        # Create some flags and a list for panorama stitching
        self.current_taking_panorama = False
        self.take_panorama = False
        self.error_panorama = False
        self.images = []


    def panorama_check_callback(self, joy):
        """
        Callback function for the joy topic
        """
        # Mapped to flight joystick
        if (not self.current_taking_panorama) and self.panorama_mode:
            if joy.buttons[10] == 1:
                self.take_panorama = True
    
    def get_frames_callback(self):
        """
        Callback function for the timer to update self.recieved_frames
        """
        img = self.footage_socket.recv()

        # Convert the recieved frames to a numpy array
        img = np.frombuffer(img, dtype=np.uint8)
        
        # Decode the frames to a cv2 image
        self.recieved_frames = cv2.imdecode(img, 1)

        # Publish the recieved frames to the stream topic
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.recieved_frames, "bgr8"))        


    def panorama_mode(self, servo):
        """
        Callback function from the servo topic which initiates stitching if 
        self.take_panorama is true
        """
        if self.take_panorama:
            self.current_taking_panorama = True
            self.get_logger().info("Taking panorama...")
            
            if servo.data[1]%10 == 0 and servo.data[1] != 100:
                self.images.append(self.recieved_frames)
            
            if servo.data[1] == 100:
                self.get_logger().info("Stitching frames...")
                self.error_panorama = PanoramaMaker.create_panorama(self.images, f"{datetime.now()}")
                self.current_taking_panorama = False
            
            if self.error_panorama and not self.current_taking_panorama:
                self.get_logger().info("Panorama error")
                self.error_panorama = False
                self.take_panorama = False
                self.images = []
            else:
                self.get_logger().info("Panorama created")
                self.take_panorama = False
                self.images = []

    def __del__(self):
        """Destructor for the class"""
        cv2.destroyAllWindows()

def main(args=None):
    """Driver function for the node"""
    rclpy.init(args=args)
    view_stream = ViewStream()
    
    executor = MultiThreadedExecutor()
    executor.add_node(view_stream)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    
    view_stream.destroy_node()
    rclpy.shutdown()