#! /usr/bin/env python


import cv2
import time
import cv_bridge
import rospy
import numpy as np
from matplotlib import pyplot as plt
from scipy import signal, ndimage
from sensor_msgs.msg import Image
        
global imL
global imR
### Vecorized implementation using Numpy Library ###
class DisparityMap():
    def __init__(self, numDisparities, blockSize):
        self.numDisparities = numDisparities
        self.blockSize = blockSize
    
    def census_convolution(self, center, kernel_size=(5, 5)):
        row_padding, col_padding = kernel_size[0]//2, kernel_size[1]//2
        image = cv2.copyMakeBorder(center, top=row_padding, left=col_padding, right=col_padding, bottom=row_padding, borderType=cv2.BORDER_CONSTANT, value=0)
        output = np.zeros(center.shape, dtype=np.uint8)
        r, c = center.shape
        
        bits = 0
        outputs = []
        for row in range(kernel_size[0]):
            for col in range(kernel_size[1]):                  
                output = (output << 1) | (image[row:row+r, col:col+c] >= center)
                bits += 1
                if bits%8==0 and bits!=0:
                    outputs.append(output.copy())
                    output = np.zeros(center.shape, dtype=np.uint8)
                    
        if (kernel_size[0]*kernel_size[1])%8!=0:
            outputs.append(output.copy())
        outputs = np.array(outputs)
        return outputs

    def find_difference(self, left, right, shift_val):
        left_t = left.copy()
        right_t = right.copy()
        if len(left.shape)==2:
            r, c = left.shape
            left_t[:, :c-shift_val] = left_t[:, shift_val:]
            output = np.sum(np.unpackbits(np.bitwise_xor(left_t.reshape(r*c,1), right_t.reshape(r*c,1)), axis = 1), axis=1).reshape(r, c)
        else:
            n, r, c = left_t.shape
            left_t[:, :, :c-shift_val] = left_t[:, :, shift_val:]
            output = np.sum(np.sum(np.unpackbits(np.bitwise_xor(left_t.reshape(n*r*c,1), right_t.reshape(n*r*c,1)), axis = 1), axis=1).reshape(n, r, c), axis=0)
            
        return output
        
    def CT(self, imgL, imgR, window_size):
        imgL = imgL.astype(np.float)
        imgR = imgR.astype(np.float)
        l, w = imgL.shape
        
        #find census for left and right image
        left = self.census_convolution(imgL, window_size)
        right = self.census_convolution(imgR, window_size)
        
        #Finding error for all the numDisparities
        errors = []
        for i in range(self.numDisparities):
            errors.append(self.find_difference(left, right, i))
        
        errors = np.array(errors)
        
        disparityMap = np.zeros((l, w), dtype=np.uint8)
        mid = int(self.blockSize/2)
        for i in range(mid, l-mid):
            for j in range(mid, min(w-self.numDisparities, w-mid)):
                disparityMap[i, j] = np.argmin(errors[:,i,j])
        #disparityMap = disparityMap/self.numDisparities
        return disparityMap
    
    def CT_with_MBM(self, imgL, imgR):
        imgL = imgL.astype(np.float)
        imgR = imgR.astype(np.float)
        l, w = imgL.shape
        
        kernels = [(1, 61), (61, 1), (11, 11), (3, 3)]
        census_convs = []
        for kernel_size in kernels:
            #find census for left and right image
            left = self.census_convolution(imgL, kernel_size)
            right = self.census_convolution(imgR, kernel_size)
        
            #Finding error for all the numDisparities
            errors = []
            for i in range(self.numDisparities):
                errors.append(self.find_difference(left, right, i)/(kernel_size[0]*kernel_size[1]))
            errors = np.array(errors)
            
            census_convs.append(errors)
        
        out = np.minimum(census_convs[0], census_convs[1]) 
        out = np.multiply(out, census_convs[2])
        out = np.multiply(out, census_convs[3])
        
        disparityMap = np.zeros((l, w), dtype=np.uint8)
        mid = int(self.blockSize/2)
        for i in range(mid, l-mid):
            for j in range(mid, min(w-self.numDisparities, w-mid)):
                disparityMap[i, j] = np.argmin(out[:,i,j])
        #disparityMap = disparityMap/self.numDisparities
        return disparityMap
    
    def CT_with_input_kernels(self, imgL, imgR, kernels):
        imgL = imgL.astype(np.float)
        imgR = imgR.astype(np.float)
        l, w = imgL.shape
        
        census_convs = []
        for kernel_size in kernels:
            #find census for left and right image
            left = self.census_convolution(imgL, kernel_size)
            right = self.census_convolution(imgR, kernel_size)
        
            #Finding error for all the numDisparities
            errors = []
            for i in range(self.numDisparities):
                errors.append(self.find_difference(left, right, i)/(kernel_size[0]*kernel_size[1]))
            errors = np.array(errors)
            
            census_convs.append(errors)
        
        out = census_convs[0]
        for i in range(1, len(kernels)):
            out = np.multiply(out, census_convs[i])

        disparityMap = np.zeros((l, w), dtype=np.uint8)
        mid = int(self.blockSize/2)
        for i in range(mid, l-mid):
            for j in range(mid, min(w-self.numDisparities, w-mid)):
                disparityMap[i, j] = np.argmin(out[:,i,j])
        #disparityMap = disparityMap/self.numDisparities
        return disparityMap
    
    #Generating Locally Consistent Disparity Map
    def LCDM(self, disparityMap, kernel,imgL,numdisparities):
        disparityMap = disparityMap.astype(np.int64)
        output = disparityMap.copy()
        r, c = imgL.shape
        row_mid, col_mid = kernel[0]//2, kernel[1]//2
        
        for i in range(2*row_mid, r-2*row_mid):
            for j in range(2*col_mid, min(c-numdisparities-col_mid, c-2*col_mid)):
                temp = disparityMap[i-row_mid:i+row_mid+1, j-col_mid:j+col_mid+1]
                val = np.bincount(temp.reshape(1, kernel[0]*kernel[1])[0]).argmax()
                output[i, j] = val
        
        return output

def storeLeftImage(msg):
    global imL
    print("done")
    bridge = cv_bridge.CvBridge()
    imL = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

def storeRightImage(msg):
    global imR
    print("Done1")
    bridge = cv_bridge.CvBridge()
    imR = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

def main():
    rospy.init_node('stereo_to_depth')
    sub1 = rospy.Subscriber('/camera/rgb/image_raw', Image, callback=storeLeftImage, queue_size=10)
    sub2 = rospy.Subscriber('/camera1/rgb/image_raw1', Image, callback=storeRightImage, queue_size=10)

    global imL
    global imR
    print("Waiting for left and right image")
    while True:
        try:
            imL
            imR
            break
        except:
            continue
    print("Initialized left and right image")

    blocksize = 16
    numdisparities = 128
    disp = cv2.imwrite("left_image.png")
    disp = cv2.imwrite("right_image.png")
    rate = rospy.Rate(1)
    while True:
	print("started")
        imgL = cv2.cvtColor(imL,cv2.COLOR_BGR2GRAY)
        imgR = cv2.cvtColor(imR,cv2.COLOR_BGR2GRAY)
        print("gray Done")
        #disparity = DisparityMap(numDisparities=numdisparities, blockSize=blocksize)
        #disparityMap1 = disparity.CT(imgL, imgR, (11, 11))
        #disparityMap2 = disparity.LCDM(disparityMap1, (11, 11), imgL,numdisparities)
        #disparityMap2 = disparityMap2*5
        #disparityMap2 = disparityMap2.astype(np.uint8)
        #heatmap = cv2.applyColorMap(disparityMap2, cv2.COLORMAP_RAINBOW)
        cv2.imshow("Left image", imL)
        cv2.imshow("Right image", imR)
        #cv2.imshow("Disparity Map", disparityMap2)
	    #cv2.imshow("Heat Map", heatmap)
	    #cv2.imwrite("left_image.png", imR)
        #cv2.imwrite("right_image.png", imL)
        print("image calulated")
        cv2.waitKey(1000)
        rate.sleep()

    # cv2.imwrite("left_image.png", imR)
    # cv2.imwrite("right_image.png", imL)
    # rate = rospy.Rate(10)
    # while True:
    #     cv2.imshow("Left image", imL)
    #     cv2.imshow("Right image", imR)
    #     cv2.waitKey(1)
    #     rate.sleep()
    
    
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
