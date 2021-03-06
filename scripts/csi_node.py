#!/usr/bin/env python
"""
BSD 3-Clause License

Copyright (c) 2021, Mohamed Abdelkader Zahana
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
# This is an implementation of a ROS node that interfaces CSI camera (e.g. Raspberry Pi) using gstreamer and OpenCV
# Author: Mohamed Abdelkader, mohamedashraf123@gmail.com

import cv2
import rospy
import time
import yaml
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo

class CSICam:
    def __init__(self):
        # Camera/sensor ID. Used in gstreamer
        self._camID = rospy.get_param('~camID', 0)
        # Output image width
        self._imgW = rospy.get_param('~imgW', 1280)
        # Output image height
        self._imgH = rospy.get_param('~imgH', 720)
        
        # Show OpenCV image for debugging
        self._showImg=rospy.get_param('~showImg', False)

        # Image publication frequency
        self._fps = rospy.get_param('~fps', 30.0)

        self._rosRate = rospy.Rate(self._fps)

        # Desired image topic name
        self._imgTopic = rospy.get_param('~imgTopic', 'csi_camera/image_raw')
        # Desired cameraInfo topic name
        self._camInfoTopic = rospy.get_param('~camInfoTopic', 'csi_camera/camera_info')
        # Camera fram name
        self._camFrameName = rospy.get_param('~camFrameName', 'csi_camera_link')

        # Image ROS publisher
        self._imgPub = rospy.Publisher(self._imgTopic, Image)

        # Camera Info ROS publisher
        self._camInfoPub = rospy.Publisher(self._camInfoTopic, CameraInfo, queue_size=1)
        self._camInfoPubRate = 5.0 # Hz
        self._camInfoTriggerTime = 0.0 # Used to publish camInfo at desired frequency

        # Calibration file
        self._yaml_fname = rospy.get_param('~calibration_file', "")
        print("Yaml file name: {}".format(self._yaml_fname))
        self._camera_info_msg = self.yaml2CameraInfo(self._yaml_fname)

        self._cvBridge = CvBridge()

        # possible resolutions
        # 3264 x 2464 @ 21fps
        # 1280 x 720 @ 60fps
        self._camSet='nvarguscamerasrc sensor-id='+str(self._camID)+' ! video/x-raw(memory:NVMM), width=1280, height=720, framerate=60/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width='+str(self._imgW)+', height='+str(self._imgH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
        try:
            self._camObj=cv2.VideoCapture(self._camSet)
        except:
            rospy.logerr("Could not create OpenCV VideoCapture object\n")

        if not self._camObj.isOpened():
            rospy.logerr("Cannot open CSI camera\n")
            exit()

    def yaml2CameraInfo(self, yaml_fname):
        """
        Parse a yaml file containing camera calibration data (as produced by 
        rosrun camera_calibration cameracalibrator.py) into a 
        sensor_msgs/CameraInfo msg.

        Parameters
        ----------
        @param yaml_fname : str
        Path to yaml file containing camera calibration data

        Returns
        -------
        @param camera_info_msg : sensor_msgs.msg.CameraInfo
        A sensor_msgs.msg.CameraInfo message containing the camera calibration
        data
        """
        # Load data from file
        try:
            with open(self._yaml_fname, "r") as file_handle:
                calib_data = yaml.load(file_handle)
        except Exception as e:
            rospy.logwarn("Couldn't find calibration file: %s", e)
            return CameraInfo()

        # Parse
        camera_info_msg = CameraInfo()
        camera_info_msg.width = calib_data["image_width"]
        camera_info_msg.height = calib_data["image_height"]
        camera_info_msg.K = calib_data["camera_matrix"]["data"]
        camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
        camera_info_msg.R = calib_data["rectification_matrix"]["data"]
        camera_info_msg.P = calib_data["projection_matrix"]["data"]
        camera_info_msg.distortion_model = calib_data["distortion_model"]
        return camera_info_msg


    def loop(self):
        while not rospy.is_shutdown():
            # Capture frame-by-frame
            ret, cv_image = self._camObj.read()
            # if frame is read correctly, ret is True
            if not ret:
                rospy.logerr_throttle(1.0, "Can't receive frame (stream end?). Exiting ...")
                break
            # publish ROS image
            try:
                imgMsg = self._cvBridge.cv2_to_imgmsg(cv_image, "bgr8")
                imgMsg.header.stamp=rospy.Time.now()
                imgMsg.header.frame_id=self._camFrameName
                self._imgPub.publish(imgMsg)

                # Publish camera info
                self._camera_info_msg.header=imgMsg.header
                self._camInfoPub.publish(self._camera_info_msg)
            except CvBridgeError as e:
                rospy.logerr_throttle(1, "Error in CvBridge: %s", e)

            # Publish camera info
            

            # Show image, if desired (for debugging only)
            if (self._showImg):
                cv2.imshow('frame', cv_image)
                if cv2.waitKey(1) == ord('q'):
                    break

            #self._rosRate.sleep()
        rospy.logwarn("Shutting down CSI node\n")
        self._camObj.release()
        cv2.destroyAllWindows()
def main():
    rospy.init_node('csi_cam_node', anonymous=True)
    rospy.loginfo("Initializing csi_camera_node\n")
    cam = CSICam()
    try:
        cam.loop()
    except KeyboardInterrupt:
        print("[CSICam] Shutting down\n")
    cam._camObj.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
