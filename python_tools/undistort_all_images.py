

import numpy as np
import cv2
import glob, os

if 1: # fr1 dataset
    INPUT_FOLDER = '/home/feiyu/Desktop/slam/code/project/my2/dataset_images_fr1_xyz/'
    OUTPUT_FOLDER = '/home/feiyu/Desktop/slam/code/project/my2/undist/'

    camera_intrinsics = np.array([
        [517.3, 0, 325.1],
        [0, 516.5, 249.7],
        [0, 0, 1]
    ])
    camera_distortion_coefs = np.array([0.2624, -0.9531, -0.0054, 0.0026, 1.1633])

    # -- Below is calibrated by myself using OpenCV (square size = 1m)
    # [[517.30097888   0.         318.34129851]
    # [  0.         516.46655457 255.51331917]
    # [  0.           0.           1.        ]]
    # [[ 0.26363656 -0.97000512 -0.00517262  0.00237889  1.19818593]]
else:
    None

# Undistort all images
cnt_img = 0
images = sorted(glob.glob(INPUT_FOLDER+'/*.png'))
for fname in images:
    filename = fname.split('/')[-1]
    cnt_img += 1
    print("processing {}/{} image: ".format(cnt_img, len(images))+filename)
    img = cv2.imread(fname)

    # Undistort
    undistort_image = cv2.undistort(
        img, camera_intrinsics, camera_distortion_coefs)


    # Show ori and undistorted image
    if 0:
        image_show = np.concatenate((img, undistort_image), axis=1)
        cv2.imshow('image_show', image_show)
        cv2.waitKey(1000)
        
    assert undistort_image.shape[0]==img.shape[0] and undistort_image.shape[1]==img.shape[1]
    cv2.imwrite(OUTPUT_FOLDER+filename, undistort_image)

