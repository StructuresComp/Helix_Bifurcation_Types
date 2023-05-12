import numpy as np
from collections import OrderedDict
from scipy.spatial import distance as dist


"""
    Source: https://www.pyimagesearch.com/2018/07/23/simple-object-tracking-with-opencv/
"""


class CentroidTracker:
    def __init__(self, obj_ids):
        self.objects = OrderedDict()
        self.occluded = [False, False]

        self.obj_ids = obj_ids
        self.initialized = False

    def register(self, centroids):
        for i, c in enumerate(centroids):
            self.objects[self.obj_ids[i]] = c

    @staticmethod
    def convert_bb_to_centroid(rects):
        # initialize an array of input centroids for the current frame
        inputCentroids = np.zeros((len(rects), 2), dtype="int")
        # loop over the bounding box rectangles
        for (i, (startX, startY, endX, endY)) in enumerate(rects):
            # use the bounding box coordinates to derive the centroid
            cX = int((startX + endX) / 2.0)
            cY = int((startY + endY) / 2.0)
            inputCentroids[i] = (cX, cY)
        return inputCentroids

    def update(self, rects, debug=False):
        inputCentroids = self.convert_bb_to_centroid(rects)
        if not self.initialized:
            if len(rects) != 2:
                return self.objects, self.occluded
            self.register(inputCentroids)
            self.initialized = True
            return self.objects, self.occluded

        if len(rects) == 0:
            self.occluded[0] = True
            self.occluded[1] = True
            return self.objects, self.occluded

        objectCentroids = list(self.objects.values())
        D = dist.cdist(np.array(objectCentroids), inputCentroids)

        if len(rects) == 1:
            i = D.argmin()
            j = 0 if i == 1 else 1
            # if np.sum((self.objects[self.obj_ids[i]] - inputCentroids[0])**2) < \
            #    np.sum((self.objects[self.obj_ids[j]] - inputCentroids[0])**2):
            if np.all(np.abs(self.objects[self.obj_ids[j]] - inputCentroids[0]) > 10):
                self.objects[self.obj_ids[i]] = inputCentroids[0]
            self.occluded[j] = True
            self.occluded[i] = False
        else:
            best = [np.inf, np.inf]
            for i, d in enumerate(D):
                new = d.min()
                if new < best[i]:
                    best[i] = new
                    self.objects[self.obj_ids[i]] = inputCentroids[d.argmin()]
            self.occluded[0] = False
            self.occluded[1] = False

            # firstD, secondD = D
            # fi, fnew = firstD.argmin()
            # si, snew = secondD.argmin()
            # if fi != si or (fi == si and firstD.min() < secondD.min()):
            #     self.objects[fi] = fnew
            #     self.objects[si] = snew
            # else:
            #     self.objects[fi] = snew
            #     self.objects[si] = fnew

        return self.objects, self.occluded
