import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt

#Import video
cap = cv.VideoCapture("testvideo7.mp4")
#cap = cv.VideoCapture(1)
width = int(cap.get(3))
height =int(cap.get(4))
print("Vid dimentions: ",width,"x",height)

#Cropping parameters
crop = [int(width*0.05),int(width*0.6)]
tunnel = [int(height*0.34),int(width*0.45),int(height*0.6),int(width*0.6)]

#Video writer
fourcc = cv.VideoWriter_fourcc('M','J','P','G')
out = cv.VideoWriter('output.avi',fourcc, 30, (width,height))

#Recording data
rect_locations = []


#Functions
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
    
def valid_rect(w,h,min_area,max_area,min_ratio,max_ratio,carea,ca_th):
    w,h = int(w),int(h)
    area = w*h
    #print(area,carea)
    if area>0 and area >= min_area and area <= max_area and carea>area*ca_th:
        
        ratio = max(w/h,h/w)
        #print(area,carea,ratio)
        if ratio>=min_ratio and ratio<=max_ratio :
            return True
    return False



#Main loop in video
while(cap.isOpened()):
    ret,frame = cap.read()
    #Quit if no frame
    if frame is None:
        break
    
    #Save video
    out.write(frame)
    
    #Display output
    output = frame+0
    
    #Crop frame
    frame[:,crop[1]:width,:] = 0
    frame[:,0:crop[0],:] = 0
    frame[tunnel[0]:tunnel[2],tunnel[1]:tunnel[3],:] = 0   


    #Tresholding for target
    mask_th_tgt = linrgb(frame,[-2,4,-2])
    mask_th_tgt = cv.medianBlur(mask_th_tgt,5)
    thresh_lower = 128
    cv.imshow('Mask_t',mask_th_tgt)
    _,mask_th_tgt = cv.threshold(mask_th_tgt,thresh_lower,255,cv.THRESH_BINARY)
    contours,_ = cv.findContours(mask_th_tgt,cv.RETR_TREE,cv.CHAIN_APPROX_NONE)
    for contour in contours:
        (x,y,w,h) = cv.boundingRect(contour)
        if cv.contourArea(contour)>20:
            cv.rectangle(output,(x,y),(x+w,y+h),(0,255,255),3)

    
    
    
    
    #Tresholding for rectangle on AGV
    '''
    hsv_th_l = np.array([120,0,0])
    hsv_th_u = np.array([255,255,255])
    mask_th_agv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    #cv.imshow('Mask_agv',mask_th_agv)
    mask_th_agv = cv.inRange(mask_th_agv,hsv_th_l,hsv_th_u)
    '''
    mask_th_agv = linrgb(frame,[2,-4,2,-30])
    cv.imshow('Mask_agv',mask_th_agv)
    _,mask_th_agv = cv.threshold(mask_th_agv,0,255,cv.THRESH_BINARY)
    mask_th_agv = cv.medianBlur(mask_th_agv,11)
    contours,_ = cv.findContours(mask_th_agv,cv.RETR_TREE,cv.CHAIN_APPROX_NONE)
    
    for contour in contours:
        #Find bounding rectangle
        rect = cv.minAreaRect(contour)
        (cx,cy), (w,h), rot_angle = rect
        box = np.int0(cv.boxPoints(rect))
        
        #Draw box
        if valid_rect(w,h,min_area=300,max_area=700,min_ratio=2,max_ratio=3.5,carea=cv.contourArea(contour),ca_th=0.75):       
            cv.drawContours(output, [box], 0, (0,255,0), 3)
            if(w<h):
                rot_angle+=90
            
            #Writing data
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
cv.destroyAllWindows()


#Plotting recorded data
rect_locations = np.array(rect_locations)
if rect_locations.ndim == 2:
    center_x = rect_locations[:,0]
    center_y = rect_locations[:,1]
    center_x = np.convolve(center_x,np.ones(5),'valid')
    center_y = np.convolve(center_y,np.ones(5),'valid')
    plt.plot(center_x,center_y)
    plt.show()
    plt.plot(rect_locations[:,2])
    plt.show()
	