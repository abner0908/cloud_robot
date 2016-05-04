# USAGE
# python classify.py --model models/svm.cpickle --image images/umbc_zipcode.png

# import the necessary packages
from __future__ import print_function
from sklearn.externals import joblib
from imtools import HOG, deskew, center_extent
import mahotas
import cv2


class DigitalRegnition:

    def __init__(self):
        model_path = "model/digitals/svm.cpickle"
        self.model = joblib.load(model_path)
        self.hog = HOG(orientations=18, pixelsPerCell=(10, 10),
                       cellsPerBlock=(1, 1), normalize=True)

    def recognize(self, image):
        # load the image and convert it to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # blur the image, find edges, and then find contours along
        # the edged regions
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edged = cv2.Canny(blurred, 30, 150)
        (cnts, _) = cv2.findContours(edged.copy(),
                                     cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # sort the contours by their x-axis position, ensuring
        # that we read the numbers from left to right
        cnts = sorted([(c, cv2.boundingRect(c)[0])
                       for c in cnts], key=lambda x: x[1])

        # loop over the contours
        length = 50
        dx, dy = 0, 0
        for (c, _) in cnts:
            # compute the bounding box for the rectangle
            (x, y, w, h) = cv2.boundingRect(c)

            # if the width is at least 7 pixels and the height
            # is at least 20 pixels, the contour is likely a digit
            if w >= 7 and h >= 20:
                # crop the ROI and then threshold the grayscale
                # ROI to reveal the digit
                roi = gray[y:y + h, x:x + w]
                thresh = roi.copy()
                T = mahotas.thresholding.otsu(roi)
                thresh[thresh > T] = 255
                thresh = cv2.bitwise_not(thresh)

                # deskew the image center its extent
                thresh = deskew(thresh, 20)
                thresh = center_extent(thresh, (20, 20))

                #cv2.imshow("thresh", thresh)

                # extract features from the image and classify it
                hist = self.hog.describe(thresh)
                hist = hist.reshape(1, -1)
                digit = self.model.predict(hist)[0]

                # draw a rectangle around the digit, the show what the
                # digit was classified as
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 1)
                cv2.putText(image, str(digit), (x - 10, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)

                cv2.rectangle(image, (dx, dy),
                              (dx + length, dy + length), (0, 0, 255), 2)
                cv2.putText(image, str(digit), (dx + 15, dy + 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)
                dx = dx + length
        return image


if __name__ == "__main__":
    dr = DigitalRegnition()
    image_path = "images/umbc_zipcode.png"
    image = cv2.imread(image_path)
    img_recognized = dr.recognize(image)
    cv2.imshow("image", img_recognized)
    cv2.waitKey(0)
