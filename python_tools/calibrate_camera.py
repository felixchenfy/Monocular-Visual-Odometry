#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import glob # for getting files' names in the disk
# import pickle # for saving variables to disk

IMAGE_FOLDER='/home/feiyu/Desktop/slam/my_vo/my2/data/fr1_rgb_calibration/'


# termination criteria
CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....
CHECKER_COLS=8
CHECKER_ROWS=6
CHESSBOARD_SQUARE_SIZE=1.0 # This doesn't affect the calibration

objp = np.zeros(( CHECKER_COLS* CHECKER_ROWS,3), np.float32)
objp[:,:2] = np.mgrid[0: CHECKER_ROWS,0: CHECKER_COLS].T.reshape(-1,2)
objp=objp*CHESSBOARD_SQUARE_SIZE

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob(IMAGE_FOLDER+'/*.png')

cnt_img=0
for fname in images:
    cnt_img+=1
    print("processing {}/{} image ...".format(cnt_img, len(images)))
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, ( CHECKER_ROWS, CHECKER_COLS),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        
        objpoints.append(objp)
    
        corners_refined = cv2.cornerSubPix(gray,corners,(7,7),(-1,-1),CRITERIA)
        imgpoints.append(corners_refined)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, ( CHECKER_ROWS, CHECKER_COLS), corners_refined,ret)
        # if 1:
        #     cv2.imshow('img',img)
        #     cv2.waitKey(1000)

cv2.destroyAllWindows()

# Calibrate camera
projection_error, camera_intrinsics, camera_distortion_coefs, rvecs, tvecs = \
    cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
print projection_error # What is the standard of this value? 
print camera_intrinsics
print camera_distortion_coefs

