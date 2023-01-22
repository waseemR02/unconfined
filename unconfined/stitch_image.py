from cv2 import Stitcher_create
import cv2
import imutils
import numpy as np


class PanoramaMaker:
    def __init__(self):
        self.image_stitcher = Stitcher_create()

    def create_panorama(self, images, title):
        error, output = self.image_stitcher.stitch(images)

        format_image = self.format_image(output)
        if not error:
            cv2.imwrite(f"{title}.jpg", format_image)
            return True
        else:
            return error

    def format_image(self, image):
        """Format image by finding contours and setting proper borders"""
        stitched_img = cv2.copyMakeBorder(
            image, 10, 10, 10, 10, cv2.BORDER_CONSTANT, (0, 0, 0))

        # convert the stitched image to grayscale and threshold it
        gray = cv2.cvtColor(stitched_img, cv2.COLOR_BGR2GRAY)
        thresh_img = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)[1]

        # find all external contours in the threshold image then find the
        # *largest* contour which will be the contour/outline of the stitched image
        contours = cv2.findContours(
            thresh_img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # grab the largest contour and are of the image
        contours = imutils.grab_contours(contours)
        areaOI = max(contours, key=cv2.contourArea)

        # create a mask for the contour and then create a masked input image
        mask = np.zeros(thresh_img.shape, dtype="uint8")
        x, y, w, h = cv2.boundingRect(areaOI)
        cv2.rectangle(mask, (x, y), (x + w, y + h), 255, -1)

        # find the minumum rectangular region that encloses the contour
        minRectangle = mask.copy()
        sub = mask.copy()

        # loop until there are no non-zero pixels left in the subtracted image
        while cv2.countNonZero(sub) > 0:
            minRectangle = cv2.erode(minRectangle, None)
            sub = cv2.subtract(minRectangle, thresh_img)

        # find contours in the minimum rectangular region and then extract the
        # bounding box (x, y)-coordinates
        contours = cv2.findContours(
            minRectangle.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        areaOI = max(contours, key=cv2.contourArea)

        x, y, w, h = cv2.boundingRect(areaOI)

        # use the bounding box coordinates to extract the our final stitched image
        output = stitched_img[y:y + h, x:x + w]

        return output