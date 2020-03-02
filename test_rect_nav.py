import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import telnetlib

#------------Initialisation-----------#

#Import video
#cap = cv.VideoCapture("test_cir.avi")
#cap = cv.VideoCapture("output2.avi")
cap = cv.VideoCapture(0)
width = int(cap.get(3))
height =int(cap.get(4))
print("Vid dimentions: ",width,"x",height)

#Video writer
fourcc = cv.VideoWriter_fourcc('M','J','P','G')
out = cv.VideoWriter('output.avi',fourcc, 30, (width,height))

#Connect to arduino via telnetlib
ip = "192.168.43.224"
port = 23
try:
    connection = telnetlib.Telnet(ip,port,5)
except:
    connection = False



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
markerH = {'min_area':450,'max_area':800,'min_ratio':2,'max_ratio':4.5,'ca_th':0.5}
markerV = {'min_area':200,'max_area':450,'min_ratio':2,'max_ratio':4.5,'ca_th':0.5}

#markerV = {'min_area':300,'max_area':650,'min_ratio':2,'max_ratio':4,'ca_th':0.75}
#markerH = {'min_area':80,'max_area':350,'min_ratio':2,'max_ratio':4,'ca_th':0.7}

#Navigation parameters
min_dist = 75   #Note: 3.2 pixels per cm
ang2t = 10
tol_angle = 0.1

#Init variables
agv_coords = [0,0]
rot_angle = 0
action = {'mode':'none','timer':0,'dir':1}   #Mode: none,fwd,rot,stop      dir: 0:fwd, 1:rev
motor = [0,0,0,0]   #:[Left motor On, Left motor reverse, Right motor on, Right motor reverse]
target_coords = [0,0]
targets = []


#------------Functions-----------#

#Returns angle between two vectors
def angle(v1,v2):
    return np.arctan2(np.cross(v2,v1),np.dot(v2,v1))
    
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

def draw_visuals(output,agv_coords,rot_angle):
    #Box for agv
    rect_agv = (
        (agv_coords[0]+agv['dof']*np.cos(rot_angle),agv_coords[1]+agv['dof']*np.sin(rot_angle)),
        (agv['h'],agv['w']),rot_angle * 180/np.pi)
    box_agv = np.int0(cv.boxPoints(rect_agv))
        
        
    #Box for gripper area
    rect_grp = (
        (agv_coords[0]+grp['dof']*np.cos(rot_angle),agv_coords[1]+grp['dof']*np.sin(rot_angle)),
        (grp['h'],grp['w']),rot_angle * 180/np.pi)
    box_grp = np.int0(cv.boxPoints(rect_grp))

    #Line
    line_length = 200
    centre_point = (int(agv_coords[0]),int(agv_coords[1]))
    line_point = (int(agv_coords[0]+line_length*np.cos(rot_angle)),int(agv_coords[1]+line_length*np.sin(rot_angle)))
    
    #Draw boxes
    cv.circle(output,centre_point,5,(128,128,0),10)
    cv.drawContours(output,[box_agv],0,(255,255,0),3)
    cv.drawContours(output,[box_grp],0,(255,255,0),3)
    cv.line(output,centre_point,line_point,(0,0,255),1)
    
    #Draw cropped area
    cv.rectangle(output,(tunnel[1],tunnel[0]),(tunnel[3],tunnel[2]),(255,0,0),3)
    cv.rectangle(output,(crop[0],0),(crop[1],width),(255,0,0),3)


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
    targets = []
    for contour in contours:
        (x,y,w,h) = cv.boundingRect(contour)
        if cv.contourArea(contour)>20:
            tgt_count += 1
            cv.rectangle(output,(x,y),(x+w,y+h),(0,255,255),3)
            targets.append([int(x+w/2),int(y+h/2)])
    
    #Sort and use leftmost
    targets = sorted(targets, key=lambda x:x[1])
    if len(targets)>0:
        target_coords = targets[0]
    
    
    #-----Tresholding for AGV------#
    
    mask_th_agv = linrgb(frame,[2,-4,2,0])
    mask_th_agv = cv.medianBlur(mask_th_agv,5)
    cv.imshow('Mask_agv',mask_th_agv)
    _,mask_th_agv = cv.threshold(mask_th_agv,30,255,cv.THRESH_BINARY)
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
        rect_marker_h = markers_H[0]
        
        #Update Rot angle and coords
        rot_angle = np.arctan2(rect_marker_h[0][1]-rect_marker_v[0][1],rect_marker_h[0][0]-rect_marker_v[0][0])
        agv_coords = [int(rect_marker_v[0][0]),int(rect_marker_v[0][1])]
        
        #Recording data
        v_ratio = max(rect_marker_v[1][0]/rect_marker_v[1][1],rect_marker_v[1][1]/rect_marker_v[1][0])
        h_ratio = max(rect_marker_h[1][0]/rect_marker_h[1][1],rect_marker_h[1][1]/rect_marker_h[1][0])
        area_v = rect_marker_v[1][0]*rect_marker_v[1][1]
        area_h = rect_marker_h[1][0]*rect_marker_h[1][1]
        rect_infos.append([v_ratio,h_ratio])  
        
    else:
        print("problem")
        print("V:",len(markers_V)," H:",len(markers_H))
    
    #Drawing visuals
    draw_visuals(output,agv_coords,rot_angle)
    cv.line(output,tuple(agv_coords),tuple(target_coords),(0,128,255),1)
    
    #-----Navigation------#
    
    #Target angle
    agv_to_target = np.array(target_coords)-np.array(agv_coords)
    distance_to_target = np.linalg.norm(agv_to_target)
    diff_angle = angle(agv_to_target,[np.cos(rot_angle),np.sin(rot_angle)])
    
    #Break events
    if distance_to_target < min_dist:
        action['mode'] = 'stop'
        motor = [0,0,0,0]
        #Done .... What next?
    
    #Reset action if timer is zero
    if action['timer'] == 0 and action['mode'] in ['fwd','rot']:
        action['mode'] = 'none'
        action['timer'] = 10
        motor = [0,0,0,0]
    
    
    if abs(diff_angle) > tol_angle and action['mode'] != 'rot':
        action['mode'] = 'none'
        action['timer'] = 10
    
    #If no action, take new action
    if action['mode'] == 'none' or action['timer'] <= 0:
        #Insert pathfinding algorithm here
        '''
        
        if target in dead zone right and robot not in right side:
            Rotate 90 degrees
            Then go forward 30cm
        elif target in dead zone left and robot not in left side:
            Rotate -90 degrees
            Then go forward 30cm
        '''
        
        if abs(diff_angle) < tol_angle:
            action['mode'] = 'fwd'
            action['timer'] = 10000         #-1 = do forever
            action['dir'] = 0
        else:
            #Rotate to target
            action['mode'] = 'rot'
            action['timer'] = int(abs(diff_angle) * ang2t)
            action['dir'] = (1-np.sign(diff_angle))/2
    
    print(action,diff_angle,distance_to_target)
        
        
    #Set velocity for forward and rotation
    if action['mode'] == 'fwd':
        motor = [1,action['dir'],1,action['dir']]
    if action['mode'] == 'rot':
        motor = [1,action['dir'],1,1-action['dir']]
    
    #Decrement timer
    action['timer'] -= 1
    
    #-----Send signal to arduino------#
    motor = [str(int(i)) for i in motor]
    if connection != False:
        connection.write(bytes([ int( "".join(motor) ,2) ]))
    
    #-----Show output/save video------#
    
    #Write info
    rect_locs.append([agv_coords[0],agv_coords[1]])
    rect_rots.append(diff_angle)
    
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
