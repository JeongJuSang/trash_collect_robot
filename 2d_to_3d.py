#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image as msg_Image, Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import pyrealsense2 as rs2
from detection_msgs.msg import BoundingBoxes
import numpy as np
from std_msgs.msg import Float64, Int64

class ImageListener:
    def __init__(self, topic):
        self.topic = topic
        self.x = 0
        self.y = 0
        self.depth=0.0
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic, msg_Image, self.imageDepthCallback)
        self.bounding_boxes_sub = rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.bounding_boxes_callback)
        self.depth_pub = rospy.Publisher('depth', Int64, queue_size=10)
        self.x_center_pub = rospy.Publisher('x_center_value', Int64, queue_size=10)
        self.y_center_pub = rospy.Publisher('y_center_value', Int64, queue_size=10)
        self.sub_info = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.imageDepthInfoCallback)
        self.intrinsics = None

    def bounding_boxes_callback(self, msg):  
        min_y = float('inf')
        # Iterate through each bounding box in the message
        # for bounding_box in msg.boundingBoxesResults_.bounding_boxes:
        for bounding_box in msg.bounding_boxes:
            if bounding_box.Class == "bottle":
                if bounding_box.ymin < min_y:
                    # Extract the x and y coordinates of the center of the bounding box
                    self.x = bounding_box.xmin + (bounding_box.xmax - bounding_box.xmin) * 0.5 
                    self.y = bounding_box.ymin + (bounding_box.ymax - bounding_box.ymin) * 0.5
                    min_y = self.y

        if not msg.bounding_boxes:
            self.x = 0
            self.y = 0
                
        self.x_center_pub.publish(int(self.x))
        self.y_center_pub.publish(int(self.y))


        return self.x, self.y
    
    # def imageDepthCallback(self, data):
    #     try:
    #         cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
    #         pix = (data.width//2, data.height//2)
    #         sys.stdout.write('%s: Depth at center(%d, %d): %f(mm)\r' % (self.topic, pix[0], pix[1], cv_image[pix[1], pix[0]]))
    #         sys.stdout.flush()
    #         if self.intrinsics:
    #             depth = cv_image[pix[1], pix[0]]
    #             result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
    #             print ('result:', result)
    #             # sys.stdout.write('%s: Depth at center(%d, %d): %f(mm)\r' % (self.topic, pix[0], pix[1], cv_image[pix[1], pix[0]]))
    #             # sys.stdout.flush()

    #     except CvBridgeError as e:
    #         print(e)
    #         return

    def imageDepthCallback(self, data):
        self.x
        self.y
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            pix = (int(self.x), int(self.y))
            sys.stdout.write('%s: bounding box Depth at center(%d, %d): %f(mm)\r' % (self.topic, pix[0], pix[1], cv_image[pix[1], pix[0]]))
            sys.stdout.flush()
            if self.intrinsics:
                self.depth = cv_image[pix[1], pix[0]]
                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], self.depth)
                print ('result:', result)
                # sys.stdout.write('%s: Depth at center(%d, %d): %f(mm)\r' % (self.topic, pix[0], pix[1], cv_image[pix[1], pix[0]]))
                # sys.stdout.flush()
                self.depth_pub.publish(int(self.depth))


        except CvBridgeError as e:
            print(e)
            return


    def imageDepthInfoCallback(self, cameraInfo):
        try:
            # import pdb; pdb.set_trace()
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
        except CvBridgeError as e:
            print(e)
            return

def main():
    topic = '/camera/depth/image_rect_raw'
    listener = ImageListener(topic)


    rospy.spin()

if __name__ == '__main__':
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    main()