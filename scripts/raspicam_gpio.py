#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge

from picamera.array import PiRGBArray
from picamera import PiCamera
import RPi.GPIO as GPIO
import cv2

class RaspiCam:
    def __init__(self):
        # ROS parameters
        self.in_channel = rospy.get_param("~gpio_in")
        self.width = rospy.get_param("~width")
        self.height = rospy.get_param("~height")
        self.framerate = rospy.get_param("~framerate")
        self.frame_id = rospy.get_param("~camera_frame_id")
        self.camera_info_url = rospy.get_param("~camera_info_url")
        self.bridge = CvBridge()

        # ROS publishers
        self.image_pub = rospy.Publisher("image_raw", Image, queue_size=1)
        self.caminfo_pub = rospy.Publisher("camera_info", CameraInfo, queue_size=1)

        # ROS subscribers
        self.trigger_sub = rospy.Subscriber("/calibration_pod/raspicam_gpio_trigger/trigger", Int8, self.trigger_camera, queue_size=1)

        # set the GPIO mode
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.in_channel, GPIO.IN)
        GPIO.add_event_detect(self.in_channel, GPIO.RISING, callback=self.trigger_camera)

        # Set camera info
        self.set_camera_info()

        # Start streaming
        self.start_camera()

    def trigger_camera(self, channel):
        # Read frame
        stream = self.camera.capture(self.rawCapture, format="bgr", use_video_port=True)
        time = rospy.Time.now()
        frame = self.rawCapture.array
        self.rawCapture.truncate(0)

        # Compose Image msg and publish
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        header = Header()
        header.stamp = time
        header.frame_id = self.frame_id
        img_msg.header = header

        # Compose Camera Info msg and publish
        self.camera_info_msg.header = header

        self.image_pub.publish(img_msg)
        self.caminfo_pub.publish(self.camera_info_msg)

    def start_camera(self):
        self.camera = PiCamera()
        self.camera.resolution = (self.width, self.height)
        self.camera.framerate = self.framerate
        self.rawCapture = PiRGBArray(self.camera)
        self.camera.start_preview()
        # camera warmup
        rospy.sleep(3.0)
        self.camera.shutter_speed = self.camera.exposure_speed
        self.camera.exposure_mode = "off"
        self.camera.awb_mode = "auto"
        rospy.loginfo("Camera is ready!")
        rospy.loginfo("Waiting for triggers")

    def set_camera_info(self):
        msg = CameraInfo()
        msg.height = self.camera_info_url["image_height"]
        msg.width = self.camera_info_url["image_width"]
        msg.distortion_model = self.camera_info_url["camera_model"]
        msg.D = self.camera_info_url["distortion_coefficients"]["data"]
        msg.K = self.camera_info_url["camera_matrix"]["data"]
        msg.R = self.camera_info_url["rectification_matrix"]["data"]
        msg.P = self.camera_info_url["projection_matrix"]["data"]
        self.camera_info_msg = msg

    def on_shutdown(self):
        self.camera.stop_preview()
        rospy.loginfo("Stopping the camera")
        GPIO.cleanup()

def main():
    node_name = "raspicam_node_gpio"
    rospy.init_node(node_name, anonymous=False)

    rc = RaspiCam()

    # On shutdown
    rospy.on_shutdown(rc.on_shutdown)
    rospy.spin()

if __name__=="__main__":
    main()
