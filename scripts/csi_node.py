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
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class CSICam:
    def __init__(self):
        # Camera/sensor ID. Used in gstreamer
        self._camID = rospy.get_param('~camID', 0)
        # Output image width
        self._imgW = rospy.get_param('~imgW', 800)
        # Output image height
        self._imgH = rospy.get_param('~imgH', 600)
        
        # Show OpenCV image for debugging
        self._showImg=rospy.get_param('showImg', False)

        # Image publication frequency
        self._fps = rospy.get_param('~fps', 30.0)
        if (self._fps > 60.0):
            rospy.logwarn("Image publication frequency is capped at 60 Hz\n")
            self._fps=60.0
        self._rosRate = rospy.Rate(self._fps)

        # Desired image topic name
        self._imgTopic = rospy.get_param('~imgTopic', 'csi_camera/image_raw')
        # Desired cameraInfo topic name
        self._camInfoTopic = rospy.get_param('~camInfoTopic', 'csi_camera/camera_info')

        # Image ROS publisher
        self._imgPub = rospy.Publisher(self._imgTopic, Image)

        # TODO Get camera intrinsic params

        self._cvBridge = CvBridge()


        self._camSet='nvarguscamerasrc sensor-id='+str(self._camID)+' ! video/x-raw(memory:NVMM), width=3264, height=2464, framerate=21/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width='+str(self._imgW)+', height='+str(self._imgH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
        try:
            self._camObj=cv2.VideoCapture(self._camSet)
        except:
            rospy.logerr("Could not create OpenCV VideoCapture object\n")

        if not self._camObj.isOpened():
            rospy.logerr("Cannot open CSI camera\n")
            exit()


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
                self._imgPub.publish(self._cvBridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                print(e)
            # SHow image, if desired (for debugging only)
            if (self._showImg):
                cv2.imshow('frame', cv_image)
                if cv2.waitKey(1) == ord('q'):
                    break

            self._rosRate.sleep()
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