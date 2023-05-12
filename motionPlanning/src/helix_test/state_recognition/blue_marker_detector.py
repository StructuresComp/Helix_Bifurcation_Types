#!/usr/bin/env python
import sys
import rospy
import message_filters
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sklearn.cluster import DBSCAN
from utils.centroid_tracker_cap import CentroidTracker
from numpy import linalg as LA
from geometry_msgs.msg import Point


class image_converter(object):
    """docstring for ."""

    def __init__(self):
        self.bridge = CvBridge()
        image_sub = message_filters.Subscriber("/camera/color/image_raw", Image, queue_size = 10)
        depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, queue_size=10)

        self.ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 1, 0.01)
        self.frame = None

        cam_info = rospy.wait_for_message("/camera/color/camera_info", CameraInfo, 1)
        self.fx = cam_info.K[0]
        self.fy = cam_info.K[4]
        self.cx = cam_info.K[2]
        self.cy = cam_info.K[5]

        print(self.fx, self.fy)

        self.marker_classifier = DBSCAN(eps=15.0, min_samples=50)

        self.coord_classifier = DBSCAN(eps=0.02, min_samples = 10)

        self.ts.registerCallback(self.callback)
        # self.mean_p0 = np.array([94, 245], dtype = np.int)
        self.mean_p0 = np.array([253, 132], dtype = np.int)
        self.mean_p0 = np.array([225, 163], dtype = np.int)

        self.node0 = np.array([-0.07451676, -0.15880916, 0.70371074], dtype=np.float)
        self.node0 = np.array([-0.24833053388577275, 0.019188988616562565, 0.8568749999999999], dtype = np.float)
        self.frame = None
        self.pub_mn = rospy.Publisher('/middleNode', Point, queue_size = 10)
        self.pub_mp = rospy.Publisher('/middlePixel', Point, queue_size = 10)

        self.pub_fig = rospy.Publisher('/figprocessed', Image, queue_size = 10)



    def callback(self, ros_image, ros_depth):
        image = depth = None
        try:
            image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            depth = self.bridge.imgmsg_to_cv2(ros_depth, "32FC1")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        image = image[:, 200:]
        depth = depth[:, 200:]

        blue_filter = self.color_detection(image)
        blue_bb, cp = self.marker_detection(blue_filter)
        # print(blue_bb)


        for i in range(len(blue_bb)):
            centroid = blue_bb[i]
            candidates = cp[i]
            if LA.norm(centroid - self.mean_p0) < 100:
                color = (0, 255, 0)
                cv2.circle(image, (centroid[1], centroid[0]), 10, color, 2)
                mean_p = np.mean(candidates, axis = 0).astype(np.int)
                # candidates = self.selectCandidates(candidates, centroid)
                dd = self.selectDepth(candidates, depth)

                centerNode = self.convert_2D_to_3D(centroid, depth,  dd)
                # centerNode = self.select3DCoord(coord3D)
                # get the most reasonable depth
                if not centerNode:
                    print("detect nothing")
                else:
                    centerNode = np.asarray(centerNode).reshape((3,))
                    if abs(centerNode[2] - self.node0[2]) < 0.25:
                        self.node0 = centerNode
                    else:
                        print("did not find center node in neighboring region.")

                self.mean_p0 = centroid.copy()
                break;
        coordM = Point()
        coordM.x = self.node0[0]
        coordM.y = self.node0[1]
        coordM.z = self.node0[2]
        self.pub_mn.publish(coordM)

        pixelM = Point()
        pixelM.x = self.mean_p0[0]
        pixelM.y = self.mean_p0[1]
        self.pub_mp.publish(pixelM)

        image_message = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.pub_fig.publish(image_message)


        # get 3D position
        cv2.imshow("ori window", image)
        # cv2.imshow("filter window", blue_filter)
        cv2.waitKey(3)

    def selectDepth(self, candidates, depth):
        zCoord = []
        for v, u in candidates:
            z = depth[int(v), int(u)] / 1000  # convert from mm to m
            zCoord.append(z)

        zCoord = np.asarray(zCoord)
        zCoord = zCoord[zCoord > 0]
        a = zCoord.reshape(-1, 1)
        clusters = self.coord_classifier.fit(zCoord.reshape(-1, 1))
        labels = np.unique(clusters.labels_)
        centerDepth = []
        for i in range(len(labels)):
            cp = zCoord[np.where(clusters.labels_ == i)]
            mean_p = np.mean(cp, axis = 0)
            centerDepth.append(mean_p)

        diffDepth = abs(centerDepth - self.node0[2])
        idx = np.argmin(diffDepth)

        return centerDepth[idx]

    def select3DCoord(self, coord):
        clusters = self.coord_classifier.fit(coord)
        labels = np.unique(clusters.labels_)
        num_unique = len(np.unique(clusters.labels_))
        if -1 in labels: num_unique -= 1

        centerNode = []

        for i in range(num_unique):
            cp = coord[np.where(clusters.labels_ == i)]
            mean_p = np.mean(cp, axis = 0)
            centerNode.append(mean_p)
            # print(mean_p)

        return centerNode

    def marker_detection(self, color_filter_img):
        x_dim, y_dim = color_filter_img.shape[1], color_filter_img.shape[0]
        img = np.swapaxes(color_filter_img, 0, 1)  # make X and Y the first and second dims, respectively
        img = img.reshape(-1, img.shape[-1])
        candidates = np.where((img != [0, 0, 0]).all(axis=1))[0]
        num_candidates = len(candidates)
        bounding_boxes = []
        CP = []
        # print(candidates)

        if num_candidates > 0:
            X = candidates % y_dim
            Y = (candidates / y_dim).astype(np.int32)
            filtered_pixels = np.zeros((num_candidates, 2))
            filtered_pixels[:, 0] = X
            filtered_pixels[:, 1] = Y

            clusters = self.marker_classifier.fit(filtered_pixels)

            labels = np.unique(clusters.labels_)
            num_unique = len(np.unique(clusters.labels_))

            if -1 in labels: num_unique -= 1
            for i in range(num_unique):
                cp = filtered_pixels[np.where(clusters.labels_ == i)]
                mean_p = np.mean(cp, axis = 0).astype(np.int)
                # print(mean_p)
                # mean_p = mean_p[[1,0]]
                cp = cp.astype(np.int)
                # print(mean_p)
                # cp = cp[:, [1, 0]]
                # new_bounds = (cp[:, 1].min(), cp[:, 0].min(), cp[:, 1].max(), cp[:, 0].max())
                bounding_boxes.append(mean_p)
                CP.append(cp)
        return bounding_boxes, CP

    def convert_2D_to_3D(self, coord, depth, d = 0):
        coord_3D = []
        if d != 0:
            x = (d / self.fx) * (coord[1] - self.cx)
            y = (d / self.fy) * (coord[0] - self.cy)
            coord_3D.append([x, y ,d])
        return coord_3D


        for v, u in coord:
            z = depth[int(v), int(u)] / 1000  # convert from mm to m
            if z == 0:
                continue
            x = (z / self.fx) * (u - self.cx)
            y = (z / self.fy) * (v - self.cy)
            coord_3D.append([x, y ,z])
        coord_3D = np.asarray(coord_3D)
        return coord_3D

    def color_detection(self, rgb):
        p_rgb = rgb.copy()
        hsv = cv2.cvtColor(p_rgb, cv2.COLOR_BGR2HSV)

        lower_blue = np.array([90, 150, 100])
        upper_blue = np.array([120, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        blue_filter_img = cv2.bitwise_and(p_rgb, p_rgb, mask=mask)
        return blue_filter_img


def main(args):
    rospy.init_node("image_converter", anonymous = True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting dow")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
