import threading
import DobotDllType as dType
import cv2
import numpy as np
from ctypes import *
import time,  platform
import tkinter.messagebox
from tkinter import *
import tkinter as tk

CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

#Load Dll
api = dType.load()

#Connect Dobot
state = dType.ConnectDobot(api, "", 115200)[0]
print("Connect status:",CON_STR[state])

if (state == dType.DobotConnect.DobotConnect_NoError):
    #Clean Command Queued
    dType.SetQueuedCmdClear(api)
    #Async Motion Params Setting
    dType.SetHOMEParams(api, 250, 0, 50, 0, isQueued = 1)
    dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
    dType.SetPTPCommonParams(api, 100, 100, isQueued = 1)
    dType.SetEndEffectorSuctionCup(api,1,0,1)
    # wait some time
    time.sleep(2)
    #dType.SetEndEffectorSuctionCup(api, 1, 0, 1)
    dType.SetQueuedCmdStartExec(api)


# open camera
camera = cv2.VideoCapture(0)
if (camera.isOpened()):
    print ('camera Open!')
else:
    print ('Fail to open!')


def Suck():
    dType.SetEndEffectorSuctionCup(api, 1, 1, 1)

def unSuck():
    dType.SetEndEffectorSuctionCup(api, 1, 0, 1)

def saveTransformMatrix(mat):
    with open('transformMatrix.txt', 'wb') as f:
        for line in mat:
            np.savetxt(f, line, fmt='%.2f')

def loadTransformMatrix():
    input = np.loadtxt("transformMatrix.txt", dtype='float', delimiter=' ')
    mat = np.mat(input)
    print('Load calibration matrix')
    return mat

width = 7
height = 6
length = 15
visionPoints = np.mat(np.zeros((3,3)))
dobotPoints = np.mat(np.zeros((3,3)))
RT = np.mat(np.zeros((3,3)))
Inv = np.mat(np.zeros((3,3)))

def calibration(corners):
    visionPoints[0, 0] = corners[0, 0, 0]
    visionPoints[1, 0] = corners[0, 0, 1]
    visionPoints[2, 0] = 1

    visionPoints[0, 1] = corners[width - 1, 0, 0]
    visionPoints[1, 1] = corners[width - 1, 0, 1]
    visionPoints[2, 1] = 1

    visionPoints[0, 2] = corners[width * height - 1, 0, 0]
    visionPoints[1, 2] = corners[width* height - 1, 0, 1]
    visionPoints[2, 2] = 1

    mat = dobotPoints * visionPoints.I
    RT[0, 0] = mat[0, 0]
    RT[1, 0] = mat[1, 0]
    RT[2, 0] = mat[2, 0]
    RT[0, 1] = mat[0, 1]
    RT[1, 1] = mat[1, 1]
    RT[2, 1] = mat[2, 1]
    RT[0, 2] = mat[0, 2]
    RT[1, 2] = mat[1, 2]
    RT[2, 2] = mat[2, 2]
    print('Save transform matrix')
    saveTransformMatrix(RT)

def transform(imgX, imgY):
    transformXY = []
    mat1 = np.mat(np.zeros((3, 1)))
    mat1[0, 0] = imgX
    mat1[1, 0] = imgY
    mat1[2, 0] = 1
    mat2 = RT * mat1
    transformXY.append(mat2[0, 0])
    transformXY.append(mat2[1, 0])
    return transformXY


def getCurrentPos():
    pos1 = dType.GetPose(api)
    return [pos1[0] , pos1[1], pos1[2]]

def goStop():
    #pos = getCurrentPos()
    #print(pos)
    #dType.SetPTPCmd(api, 1, pos[0], pos[1], 20, 0, 1)
    #unSuck()
    stopPos = [ 160 , -140 , 50 ]

    queuedCmdIndex = dType.SetPTPCmd(api, 1, stopPos[0], stopPos[1], stopPos[2], 0, 1)
    # wait until robot is finish motion
    print('Go to stop position')
    stop =0
    while (queuedCmdIndex != dType.GetQueuedCmdCurrentIndex(api)) and (stop < 10):
        stop+=1
        time.sleep(1)
    print('Ready for next wrap')


indx = 0
def graspObjectAtPoint(x,y):
    objPos = transform(x, y)
    dType.SetPTPCmd(api, 1, objPos[0], objPos[1], 20, 0, 1)
    graspHeight = -40
    dType.SetPTPCmd(api, 1, objPos[0], objPos[1],graspHeight, 0, 1)
    Suck()
    dType.SetPTPCmd(api, 1, objPos[0], objPos[1], 20, 0, 1)
    desPos = np.array([[110,-140,-45],[160,-140,-45],[200,-140,-45],[110,-190,-45],[160,-190,-45],[200,-190,-45]])
    dType.SetPTPCmd(api, 1, desPos[graspObjectAtPoint.counter % 6,0], desPos[graspObjectAtPoint.counter % 6,1], 30, 0, 1)
    dType.SetPTPCmd(api, 1, desPos[graspObjectAtPoint.counter % 6,0], desPos[graspObjectAtPoint.counter % 6,1], desPos[graspObjectAtPoint.counter % 6,2], 0, 1)
    unSuck()
    time.sleep(1)
    dType.SetPTPCmd(api, 1, desPos[graspObjectAtPoint.counter % 6, 0], desPos[graspObjectAtPoint.counter % 6, 1], 30, 0,
                    1)
    graspObjectAtPoint.counter = graspObjectAtPoint.counter + 1
    print(graspObjectAtPoint.counter)

graspObjectAtPoint.counter =0

def drawCircle(event,x,y,flags,param):
    if event==cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(img2,(x,y),10,(0,0,255), 10, -1)
        cv2.imshow('DobotRun', img2)
        cv2.waitKey(1000)
        #list = transform(x, y)
        # dType.SetPTPCmd(api, 0, list[0], list[1], -20, 0, 1)
        graspObjectAtPoint(x,y)

## Receive an image and return the list of center point for each object
def objectDetection(img):
    img2 = img
    img2 = cv2.blur(img2, (21, 21))
    img = cv2.cvtColor(img2, cv2.COLOR_RGB2HSV)
    channels = cv2.split(img)
    channels[2] = cv2.equalizeHist(channels[2])
    mergImg = cv2.merge(channels)
    lowlbBlue = (5, 100, 235)
    highlbBlue = (50, 150, 250)
    lowlbGreen = (20, 20, 150)
    highlbGreen = (90, 100, 255)  # 200
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
            center = [0, 0]
            center[0] = int(distMoment['m10'] / distMoment['m00'])
            center[1] = int(distMoment['m01'] / distMoment['m00'])
            listCenter.append(center)
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
            listCenter.append(center)

    thresholdImg = cv2.inRange(mergImg, lowlbBlue, highlbBlue)
    im2, contours, hierarchy = cv2.findContours(thresholdImg, 1, cv2.CHAIN_APPROX_NONE)

    # processing contour
    for i in range(len(contours)):
        area = cv2.contourArea(contours[i])
        if (area > areaMin) and (area < areaMax):
            distMoment = cv2.moments(contours[i])
            center = [0, 0]
            center[0] = int(distMoment['m10'] / distMoment['m00'])
            center[1] = int(distMoment['m01'] / distMoment['m00'])
            listCenter.append(center)

    listCenter = np.array(listCenter)
    return  listCenter
# Load transform matrix

RT = loadTransformMatrix()
goStop()
# Create tinker window and hide it

root = tk.Tk()
root.withdraw()

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6 * 7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

while True:
    grabbed, img = camera.read()  #Video stream by frame by frame
    if not grabbed:
        break
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  #convert grey image

    cv2.imshow('lwpCVWindow', gray)  #Display the captured video stream
    key = cv2.waitKey(1) & 0xFF
    if key == ord('c'):
        cornerCount = 0
        ret, corners = cv2.findChessboardCorners(gray,(7, 6),None)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            # Draw and display the cornes

            #red Point
            cv2.circle(img, (corners2[0, 0, 0], corners2[0, 0, 1]), 10, (0, 0 ,255), -1)
            #green Point
            cv2.circle(img, (corners2[width - 1, 0, 0], corners2[width - 1, 0, 1]), 10, (0, 255, 0), -1)
            #bluePoint
            cv2.circle(img, (corners2[width * height - 1, 0, 0], corners2[width * height - 1, 0, 1]), 10, (255, 0, 0), -1)

            img = cv2.drawChessboardCorners(img, (7, 6), corners2, ret)
            cv2.imshow('matWindow', img)
            cv2.waitKey(500)

            tkinter.messagebox.askokcancel("VisionDemo", 'Please move the Dobot to the red spot by teaching key!')
            dType.GetPose(api)
            dobotPoints[0, 0] = dType.GetPose(api)[0]
            dobotPoints[1, 0] = dType.GetPose(api)[1]
            dobotPoints[2, 0] = 1

            tkinter.messagebox.askokcancel("VisionDemo", 'Please move the Dobot to the green spot by teaching key!')
            dType.GetPose(api)
            dobotPoints[0, 1] = dType.GetPose(api)[0]
            dobotPoints[1, 1] = dType.GetPose(api)[1]
            dobotPoints[2, 1] = 1

            tkinter.messagebox.askokcancel("VisionDemo", 'Please move the Dobot to the blue spot by teaching key!')
            dType.GetPose(api)
            dobotPoints[0, 2] = dType.GetPose(api)[0]
            dobotPoints[1, 2] = dType.GetPose(api)[1]
            dobotPoints[2, 2] = 1

            calibration(corners)
            cv2.destroyWindow('matWindow')

    if key == ord("s"):
        cv2.destroyWindow('lwpCVWindow')

        # Start to Execute Command Queued
        dType.SetQueuedCmdStartExec(api)
        # _,img2 = camera.read()
        # img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2BGRA)
        cv2.namedWindow('DobotRun')
        cv2.setMouseCallback('DobotRun', drawCircle)
        hasObject = False
        considerListObject = np.array([])
        countFrame = 0
        while True:
            nothing , img2 = camera.read()
            #img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)
            cv2.imshow('DobotRun', img2)
            listObjects = objectDetection(img2)
            if len(listObjects) > 0:
                hasObject = True
                if  len(considerListObject) == 0:
                    considerListObject  = listObjects
                    countFrame = 0
                else:
                    # if we have different object number reset

                    if len(listObjects) !=  len(considerListObject):
                        print('Reset consider list object')
                        countFrame = 0
                        considerListObject = listObjects
                    else:
                        differ = np.sum(np.abs(considerListObject- listObjects))
                        print('Differ:' ,differ)
                        if differ < 50:
                            countFrame += 1
                        else:
                            print('Reset consider list object')
                            countFrame = 0
                            considerListObject = listObjects

            else:
                hasObject = False
                considerListObject = np.array([])
                countFrame = 0

            # wrap object
            if hasObject and countFrame == 3:
                print('Start wrap object!!!!!')
                print('Number objects:',len(considerListObject))
                for i in range(len(considerListObject)):
                    print('\t Wrap object at ',considerListObject[i,0] ,considerListObject[i,1])
                    graspObjectAtPoint(considerListObject[i,0],considerListObject[i,1])

                goStop()
                considerListObject = np.array([])
                countFrame = 0


            if cv2.waitKey(500) & 0xFF == 27:
                cv2.destroyWindow('DobotRun')
                break
                # Stop to Execute Command Queued
        dType.SetQueuedCmdStopExec(api)

    if key == ord('q'):
        # Disconnect Dobot
        dType.DisconnectDobot(api)
        cv2.destroyAllWindows()
        break

print('We get here')
root.quit()
sys.exit()
print('We are at the end')