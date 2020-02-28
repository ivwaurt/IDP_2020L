import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import tkinter as tk


#------------Initialisation-----------#

#Import video
#cap = cv.VideoCapture("test_cir.avi")
#cap = cv.VideoCapture("testvideo1.mp4")
cap = cv.VideoCapture(0)
width = int(cap.get(3))
height =int(cap.get(4))
print("Vid dimentions: ",width,"x",height)

#Video writer
fourcc = cv.VideoWriter_fourcc('M','J','P','G')
out = cv.VideoWriter('output.avi',fourcc, 30, (width,height))


#------------Parameters-----------#

#Recording data
rect_locs = []
rect_rots = []
rect_infos = []

#Cropping parameters
crop = [int(width*0.03),int(width*0.6)]
tunnel = [int(height*0.34),int(width*0.45),int(height*0.6),int(width*0.6)]

#Bounding box parameters
agv = {'w':65,'h':100,'dof':10}
grp = {'w':90,'h':40,'dof':70}
markerH = {'min_area':450,'max_area':800,'min_ratio':2,'max_ratio':4,'ca_th':0.75}
markerV = {'min_area':200,'max_area':450,'min_ratio':2,'max_ratio':4,'ca_th':0.75}

#markerV = {'min_area':300,'max_area':650,'min_ratio':2,'max_ratio':4,'ca_th':0.75}
#markerH = {'min_area':80,'max_area':350,'min_ratio':2,'max_ratio':4,'ca_th':0.7}

#Init variables
center = [0,0]
rot_angle = 0

#------------Functions-----------#

def meanangle(*x):
    s,c = 0,0
    for i in x:
        i *= np.pi/180
        s += np.sin(i)
        c += np.cos(i)
    return np.arctan2(s,c)*180/np.pi    
    
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
        #print(area,carea/area,ratio)
        if ratio>=min_ratio and ratio<=max_ratio :
            return True
    return False


#------------Main Loop-----------#

while(cap.isOpened()):
    
    ret,frame = cap.read()
    #Quit if no frame
    if frame is None:
        break
    
    #Save video
    #out.write(frame)
    
    #Display output
    output = frame+0
    
    #Crop frame
    frame[:,crop[1]:width,:] = 0
    frame[:,0:crop[0],:] = 0
    frame[tunnel[0]:tunnel[2],tunnel[1]:tunnel[3],:] = 0   


    #-----Tresholding for target-----#
    
    mask_th_tgt = linrgb(frame,[-2,4,-2])
    mask_th_tgt = cv.medianBlur(mask_th_tgt,5)
    thresh_lower = 128
    cv.imshow('Mask_t',mask_th_tgt)
    _,mask_th_tgt = cv.threshold(mask_th_tgt,thresh_lower,255,cv.THRESH_BINARY)
    contours,_ = cv.findContours(mask_th_tgt,cv.RETR_TREE,cv.CHAIN_APPROX_NONE)
    tgt_count = 0
    for contour in contours:
        (x,y,w,h) = cv.boundingRect(contour)
        if cv.contourArea(contour)>20:
            tgt_count += 1
            cv.rectangle(output,(x,y),(x+w,y+h),(0,255,255),3)
    #print(tgt_count)
    
    
    #-----Tresholding for AGV------#
    
    mask_th_agv = linrgb(frame,[2,-4,2,-50])
    mask_th_agv = cv.medianBlur(mask_th_agv,5)
    _,mask_th_agv = cv.threshold(mask_th_agv,0,255,cv.THRESH_BINARY)
    cv.imshow('Mask_agv',mask_th_agv)
    contours,_ = cv.findContours(mask_th_agv,cv.RETR_TREE,cv.CHAIN_APPROX_NONE)
    
    markers_V = []
    markers_H = []
    
    for contour in contours:
        #Find bounding rectangle
        rect = cv.minAreaRect(contour)
        (cx,cy), (w,h), rot_angle = rect
        if(w<h):
            rot_angle+=90
        #Test if valid rectangle
        if valid_rect(w,h,markerV['min_area'],markerV['max_area'],markerV['min_ratio'],markerV['max_ratio'],cv.contourArea(contour),markerV['ca_th']):       
            markers_V.append(rect)
            box_test = np.int0(cv.boxPoints(rect))
            cv.drawContours(output, [box_test], 0, (0,255,0), 3)

        if valid_rect(w,h,markerH['min_area'],markerH['max_area'],markerH['min_ratio'],markerH['max_ratio'],cv.contourArea(contour),markerH['ca_th']):       
            markers_H.append(rect)
            box_test = np.int0(cv.boxPoints(rect))
            cv.drawContours(output, [box_test], 0, (0,255,0), 3)
    

    #-----Updating location of the AGV------#
    
    if len(markers_V)*len(markers_H) == 1:
        #Box
        rect_marker_v = markers_V[0]
        box_v = np.int0(cv.boxPoints(rect_marker_v))
        rect_marker_h = markers_H[0]
        box_h = np.int0(cv.boxPoints(rect_marker_h))
        
        #Update Rot angle
        rot_angle = np.arctan2(rect_marker_h[0][0]-rect_marker_v[0][0],rect_marker_h[0][1]-rect_marker_v[0][1])
        
        #Update Center
        center = [rect_marker_v[0][0],rect_marker_v[0][1]]
        
        #Writing data
        v_ratio = max(rect_marker_v[1][0]/rect_marker_v[1][1],rect_marker_v[1][1]/rect_marker_v[1][0])
        h_ratio = max(rect_marker_h[1][0]/rect_marker_h[1][1],rect_marker_h[1][1]/rect_marker_h[1][0])
        area_v = rect_marker_v[1][0]*rect_marker_v[1][1]
        area_h = rect_marker_h[1][0]*rect_marker_h[1][1]
         
        rect_infos.append([area_v,area_h])    
    else:
        print("problem")
        print("V:",len(markers_V)," H:",len(markers_H))
    
    #-----Drawing visuals------#
    
    #Box for agv
    rect_agv = (
        (center[0]+agv['dof']*np.sin(rot_angle),center[1]+agv['dof']*np.cos(rot_angle)),
        (agv['w'],agv['h']),-rot_angle * 180/np.pi)
    box_agv = np.int0(cv.boxPoints(rect_agv))
        
    #Box for gripper area
    rect_grp = (
        (center[0]+grp['dof']*np.sin(rot_angle),center[1]+grp['dof']*np.cos(rot_angle)),
        (grp['w'],grp['h']),-rot_angle * 180/np.pi)
    box_grp = np.int0(cv.boxPoints(rect_grp))

    #Line
    line_length = 200
    center_point = (int(center[0]),int(center[1]))
    line_point = (int(center[0]+line_length*np.sin(rot_angle)),int(center[1]+line_length*np.cos(rot_angle)))
    
    #Write info
    rect_locs.append([center[0],center[1]])
    rect_rots.append(rot_angle)
    
    #Draw boxes
    cv.circle(output,center_point,5,(128,128,0),10)
    cv.drawContours(output,[box_agv],0,(255,255,0),3)
    cv.drawContours(output,[box_grp],0,(255,255,0),3)
    cv.line(output,center_point,line_point,(0,0,255),1)
    
    #Draw cropped area
    cv.rectangle(output,(tunnel[1],tunnel[0]),(tunnel[3],tunnel[2]),(255,0,0),3)
    cv.rectangle(output,(crop[0],0),(crop[1],width),(255,0,0),3)
    
    #-----Show output/save video------#
    
    #Show output
    cv.imshow('output',output)
    
    #Save video
    out.write(output)
    
    #Press q to quit
    if cv.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv.destroyAllWindows()
        break



#-----Terminate sequence and plot data------#

cap.release()
cv.destroyAllWindows()

#Plotting recorded data
plt.plot([i[0] for i in rect_locs],[i[1] for i in rect_locs])
plt.show()
plt.plot(rect_rots)
plt.show()
plt.plot([i[0] for i in rect_infos])
plt.plot([i[1] for i in rect_infos])
plt.show()
