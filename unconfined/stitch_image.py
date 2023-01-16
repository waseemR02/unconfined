from cv2 import Stitcher_create
import cv2

image_stitcher = Stitcher_create()

def create_panorama(images,title):
    error, output = image_stitcher.stitch(images)

    if not error:
        cv2.imwrite(f"{title}.jpg",output)
        return True
    else:
        return error