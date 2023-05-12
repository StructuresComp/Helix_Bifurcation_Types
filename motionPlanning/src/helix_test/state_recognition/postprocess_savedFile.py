#!/usr/bin/env python
import sys
import cv2
import numpy as np
from sklearn.cluster import DBSCAN
from utils.centroid_tracker_cap import CentroidTracker
from numpy import linalg as LA

class postProcess(object):
    """docstring for ."""
    def __init__(self):
        self.cap = cv2.VideoCapture('/home/dezhong/Desktop/helixBifurcation_robotic_parts_back/src/helix_test/saveVideo/filename.avi')
        self.frame = None
        # cam_info = rospy.wait_for_message("/camera/color/camera_info", CameraInfo, 1)
        # self.fx = cam_info.K[0]
        # self.fy = cam_info.K[4]
        # self.cx = cam_info.K[2]
        # self.cy = cam_info.K[5]

        self.marker_classifier = DBSCAN(eps=15.0, min_samples=50)

        self.coord_classifier = DBSCAN(eps=0.01, min_samples = 30)

        # self.ts.registerCallback(self.callback)

        self.frame = None
        self.mean_p0 = np.array([218, 181], dtype = np.int)
        self.trackedP = []

    def playback(self):
        if (self.cap.isOpened()== False):
            print("Error opening video stream or file")
        while(self.cap.isOpened()):
            ret, self.frame = self.cap.read()
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break
            self.frame = self.frame[:, 200:]
            blue_filter = self.color_detection(self.frame)
            blue_bb = self.marker_detection(blue_filter)

            for i in range(len(blue_bb)):
                centroid = blue_bb[i]
                if LA.norm(centroid - self.mean_p0) < 5:
                    color = (0, 255, 0)
                    cv2.circle(self.frame, centroid, 10, color, 2)
                    self.mean_p0 = centroid.copy()
                    break;
            self.trackedP.append(centroid)


            # centroid = blue_bb[0]
            # print(centroid)
            # color = (0, 255, 0)
            # cv2.circle(blue_filter, centroid, 10, color, 2)
            # print(blue_bb)

            cv2.imshow('color',self.frame)
            # Press Q on keyboard to  exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.cap.release()

    def color_detection(self, rgb):
        p_rgb = rgb.copy()
        hsv = cv2.cvtColor(p_rgb, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([90, 200, 100])
        upper_blue = np.array([120, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        blue_filter_img = cv2.bitwise_and(p_rgb, p_rgb, mask=mask)
        return blue_filter_img


    def marker_detection(self, color_filter_img):
        x_dim, y_dim = color_filter_img.shape[1], color_filter_img.shape[0]
        img = np.swapaxes(color_filter_img, 0, 1)  # make X and Y the first and second dims, respectively
        img = img.reshape(-1, img.shape[-1])
        candidates = np.where((img != [0, 0, 0]).all(axis=1))[0]
        num_candidates = len(candidates)
        bounding_boxes = []
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
                mean_p = mean_p[[1,0]]
                # new_bounds = (cp[:, 1].min(), cp[:, 0].min(), cp[:, 1].max(), cp[:, 0].max())
                bounding_boxes.append(mean_p)
        # print(cp)

        return bounding_boxes



    # def callback(self, ros_image, ros_depth):
    #     image = depth = None
    #     try:
    #         image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
    #         depth = self.bridge.imgmsg_to_cv2(ros_depth, "32FC1")
    #     except CvBridgeError as e:
    #         rospy.logerr("CvBridge Error: {0}".format(e))
    #
    #     blue_filter = self.color_detection(image)
    #     blue_bb, cp = self.marker_detection(blue_filter)
    #     coord = self.convert_2D_to_3D(cp, depth)
    #     self.select3DCoord(coord)
    #
    #     color = (128, 0, 0)
    #
    #     for x, y in blue_bb:
    #         text = "ID {}".format(1)
    #         # cv2.putText(image, text, (x - 15, y - 20),
    #         #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    #         cv2.circle(image, (x, y), 4, color, -1)
    #         cv2.rectangle(image, (x-15, y-15), (x+15, y+15), color, 2)
    #
    #
    #     cv2.imshow("Image window", blue_filter)
    #     cv2.imshow("ori window", image)
    #     cv2.waitKey(3)

    def select3DCoord(self, coord):
        clusters = self.coord_classifier.fit(coord)
        cp = coord[np.where(clusters.labels_ == 0)]
        centroid = np.sum(cp, axis = 0)/np.shape(cp)[0]
        if centroid[0] == 0:
            print(cp)
            print(coord[np.where(clusters.labels_ == -1)])
        # print(centroid)

    def convert_2D_to_3D(self, coord, depth):
        coord_3D = []
        for v, u in coord:
            z = depth[int(v), int(u)] / 1000  # convert from mm to m
            x = (z / self.fx) * (u - self.cx)
            y = (z / self.fy) * (v - self.cy)
            coord_3D.append([x, y ,z])
        coord_3D = np.asarray(coord_3D)
        return coord_3D


def main(args):
    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print("Shutting dow")
    pp = postProcess()
    pp.playback()
    cv2.destroyAllWindows()
    trackedP = np.asarray(pp.trackedP)
    fname = '/home/dezhong/Desktop/helixBifurcation_robotic_parts_back/src/helix_test/saveData/trackedP.txt'
    np.savetxt(fname, trackedP, ".2f%")

if __name__ == '__main__':
    main(sys.argv)
