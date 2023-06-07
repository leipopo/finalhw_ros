#!/usr/bin/env python
import cv2
import argparse
import myutils


ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True,
                    help="path to input image")
ap.add_argument("-o", "--output", required=True,
                    help="path to output image")

args = vars(ap.parse_args())

def cv_show(name, img):
    cv2.imshow(name, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
img =cv2.imread(args["template"])
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv_show("gray", gray)
gray = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
cv_show("gray", gray)
graycnt, hierarchy = cv2.findContours(gray.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

cv2.drawContours(img, graycnt, -1, (0, 0, 255), 3)
cv_show("img", img)
graycnt = myutils.sort_contours(graycnt, method="left-to-right")[0]
digits = {}
for (i, c) in enumerate(graycnt):
    (x, y, w, h) = cv2.boundingRect(c)
    roi = gray[y:y+h, x:x+w]
    roi = cv2.resize(roi, (57, 88))
    digits[i] = roi
