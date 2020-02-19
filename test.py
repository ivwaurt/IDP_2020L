import numpy as np
import cv2 as cv

#Import image
img = cv.imread("testimg4.png")
output = img

#Filter out only (blue/green/red) 
def isofilter(img,index):
    img = np.asarray(img,dtype='int16')
    out = 2*(3*img[:,:,index]-np.sum(img,axis=2))
    out = np.clip(out,a_min=0,a_max=255)
    return np.asarray(out, dtype='uint8' )

#Low pass filter
m = cv.medianBlur(img,11)

#B=0, G=1, R=2
out = isofilter(m,1)


#Draw bounding box
thresh_min = 100
_,thrash = cv.threshold(out,thresh_min,255,cv.THRESH_BINARY)
contours,_ = cv.findContours(thrash,cv.RETR_TREE,cv.CHAIN_APPROX_NONE)
for contour in contours:
    (x,y,w,h) = cv.boundingRect(contour)
    if cv.contourArea(contour)>500:
        cv.rectangle(img,(x,y),(x+w,y+h),(0,255,0),3)

#Show image
cv.imshow('img',img)
cv.imshow('output',out)
cv.waitKey(0)
cv.destroyAllWindows()
	