import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt

#Import video
cap = cv.VideoCapture("testvideo5.mp4")
#cap = cv.VideoCapture(1)
width = int(cap.get(3))
height =int(cap.get(4))
print("Vid dimentions: ",width,"x",height)

#Cropping parameters
crop = [int(width*0),int(width*0.65)]
tunnel = [int(height*0.25),int(width*0.45),int(height*0.65),int(width*0.7)]

#Video writer
fourcc = cv.VideoWriter_fourcc('M','J','P','G')
out = cv.VideoWriter('output.avi',fourcc, 30, (width,height))

#Recording data
rect_locations = []

def isofilter(img,index,bias):
    img = np.asarray(img,dtype='int16')
    out = 2*(3*img[:,:,index]-np.sum(img,axis=2))+bias
    out = np.clip(out,a_min=0,a_max=255)
    return np.asarray(out, dtype='uint8' )

def linrgb(img,a):
    if(len(a))==3:
        a.append(0)
    img = np.asarray(img,dtype='int16')
    out = a[0]*img[:,:,0]+a[1]*img[:,:,1]+a[2]*img[:,:,2]+a[3]
    out = np.clip(out,a_min=0,a_max=255)
    return np.asarray(out, dtype='uint8' )    
    
def valid_rect(w,h,min_area,max_area,min_ratio,max_ratio):
    w,h = int(w),int(h)
    area = w*h

    if area>0 and area >= min_area and area <= max_area:
        ratio = max(w/h,h/w)
        if ratio>=min_ratio and ratio<=max_ratio :
            return True
    return False



circle_centers = []
while(cap.isOpened()):
    ret,frame = cap.read()
    #Quit if no frame
    if frame is None:
        break
    
    out.write(frame)
    output = frame
    
    
    #Convert to red and blur
    mask_th = linrgb(frame,[1,-3,1])
    #mask_th = isofilter(mask_th,2,-100)
    mask_th = cv.medianBlur(mask_th,21)
    
    
    #Crop frame
    mask_th[:,crop[1]:width] = 0
    mask_th[:,0:crop[0]] = 0
    mask_th[tunnel[0]:tunnel[2],tunnel[1]:tunnel[3]] = 0
    
    #Tresholding
    _,mask_th = cv.threshold(mask_th,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
    cv.imshow('Mask',mask_th)
    contours,_ = cv.findContours(mask_th,cv.RETR_TREE,cv.CHAIN_APPROX_NONE)
    for contour in contours:
        #Find bounding rectangle
        rect = cv.minAreaRect(contour)
        (cx,cy), (w,h), rot_angle = rect
        box = np.int0(cv.boxPoints(rect))
        #print(w,h)
        
        #Draw box
        if valid_rect(w,h,min_area=750,max_area=1500,min_ratio=1,max_ratio=2):
        #if valid_rect(w,h,min_area=10000,max_area=50000,min_ratio=3,max_ratio=8):        
            cv.drawContours(output, [box], 0, (0,255,0), 3)
    
            #Writing data
            if(w<h):
                rot_angle+=90
            rect_locations.append([cx,cy,rot_angle])
    
    #Draw cropped area
    cv.rectangle(output,(tunnel[1],tunnel[0]),(tunnel[3],tunnel[2]),(255,0,0),3)
    cv.rectangle(output,(crop[0],0),(crop[1],width),(255,0,0),3)
    
    
    #Show output
    cv.imshow('output',output)
    
    #Press q to quit
    if cv.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv.destroyAllWindows()
        break

out.release()

#Plotting recorded data
center_x = np.array(rect_locations)[:,0]
center_y = np.array(rect_locations)[:,1]
center_x = np.convolve(center_x,np.ones(1),'valid')
center_y = np.convolve(center_y,np.ones(1),'valid')
plt.plot(center_x,center_y)
plt.show()
plt.plot(np.array(rect_locations)[:,2])
plt.show()
	