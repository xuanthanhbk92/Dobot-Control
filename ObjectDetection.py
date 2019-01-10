import numpy as np
from cv2 import *
import matplotlib.pyplot as plt

def showImage(img):
    imshow('win', img)
    waitKey(0)


def objectDetection(img):
    img2 = img
    img2 = cv2.blur(img2, (3, 3))
    img = cv2.cvtColor(img2, cv2.COLOR_RGB2HSV)
    channels = cv2.split(img)
    channels[2] = cv2.equalizeHist(channels[2])
    mergImg = cv2.merge(channels)
    lowlbGreen = (32, 47, 53)
    highlbGreen = (86, 171, 255)
    lowlbRedYellow = (74, 92, 211)
    highlbRedYellow = (207, 213, 255)
    thresholdImg = cv2.inRange(mergImg, lowlbRedYellow, highlbRedYellow)
    im2, contours, hierarchy = cv2.findContours(thresholdImg, 1, cv2.CHAIN_APPROX_NONE)
    areaMin = 5000
    areaMax = 15000

    listCenter = []
    for i in range(len(contours)):
        area = cv2.contourArea(contours[i])
        if (area > areaMin) and (area < areaMax):
            distMoment = cv2.moments(contours[i])
            center =[0,0]
            center[0] = int(distMoment['m10'] / distMoment['m00'])
            center[1] = int(distMoment['m01'] / distMoment['m00'])
            a  = center
            listCenter.append(a)
            print(center)
    thresholdImg = cv2.inRange(mergImg, lowlbGreen, highlbGreen)
    im2, contours, hierarchy = cv2.findContours(thresholdImg, 1, cv2.CHAIN_APPROX_NONE)

    # processing contour
    for i in range(len(contours)):
        area = cv2.contourArea(contours[i])
        if (area > areaMin) and (area < areaMax):
            distMoment = cv2.moments(contours[i])
            center = [0, 0]
            center[0] = int(distMoment['m10'] / distMoment['m00'])
            center[1] = int(distMoment['m01'] / distMoment['m00'])
            a = center
            listCenter.append(a)
            print(center)
    print(listCenter)
    listCenter = np.array(listCenter)
    return  listCenter
img = imread('12.png')
# testList = np.array([])
# testList = objectDetection(img)
#
# print('The list object is')
# print(testList)
# print(testList.shape)
# print(len(testList))
# sumList = np.array(([200,300],[300,300],[400,400]))
# print(abs(sumList - testList))
# print(np.sum(abs(sumList - testList)))



namedWindow('win',WINDOW_AUTOSIZE)
showImage(img)
img2 = blur(img,(7,7))
#showImage(img2)
img = cvtColor(img2,COLOR_RGB2HSV)
#showImage(img)
channels = split(img)
channels[2] = equalizeHist(channels[2])
mergImg = merge(channels)

cvBack = cv2.cvtColor(mergImg, COLOR_HSV2BGR)
showImage(cvBack)
showImage(mergImg)

#lowlbGreen = (20,20,150)
#highlbGreen = (90,100,255) # 200


lowlbBlue = (5,100,235)
highlbBlue = (50,150,250) # 200
#
# lowlbRedYellow = (74,92,211)
# highlbRedYellow = (207,213,255)
# thresholdImg = inRange(mergImg,lowlbRedYellow,highlbRedYellow)
# showImage(thresholdImg)
#
# im2,contours,hierarchy= findContours(thresholdImg,1,CHAIN_APPROX_NONE)
#
# # processing contour
#
areaMin = 5000
areaMax = 15000
#
# listCenter = []
# center = [ 0,0]
# for i in range(len(contours)):
#     area = contourArea(contours[i])
#     print(area)
#     if (area > areaMin) and (area < areaMax):
#         distMoment = moments(contours[i])
#         center[0] = int(distMoment['m10'] / distMoment['m00'])
#         center[1] = int(distMoment['m01']/ distMoment['m00'])
#         listCenter.append(center)
#         print(center)


# processing green
print(mergImg)
thresholdImg = inRange(mergImg,lowlbGreen,highlbGreen)
showImage(thresholdImg)
im2, contours, hierarchy = findContours(thresholdImg, 1, CHAIN_APPROX_NONE)

# processing contour

listCenter = []
center = [0, 0]
for i in range(len(contours)):
    area = contourArea(contours[i])
    print(area)
    if (area > areaMin) and (area < areaMax):
        distMoment = moments(contours[i])
        center[0] = int(distMoment['m10'] / distMoment['m00'])
        center[1] = int(distMoment['m01'] / distMoment['m00'])
        listCenter.append(center)
        print(center)

destroyWindow('win')
print('Quit application')
quit()
