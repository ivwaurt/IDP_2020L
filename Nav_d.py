import numpy as np
import cv2 as cv
import telnetlib

#------------Initialisation-----------#

#Import video
cap = cv.VideoCapture(0)
width = int(cap.get(3))
height =int(cap.get(4))
print("Vid dimentions: ",width,"x",height)

#Connect to arduino via telnetlib
ip = "192.168.43.224"
port = 23
connection = False

#Start trigger
StartTrigger = input("Enter any key")

#------------Parameters-----------#

#Cropping parameters (pixels)
crop = [19,320]               #[Left,Right]
tunnel = [163,288,288,384]    #[Left,top,right,bottom]

#Display Box params, w= width, h=height, dof = offset from center
agv = {'w':65,'h':100,'dof':10}  
grp = {'w':90,'h':40,'dof':70}

#Thresholding for rectangle detection for AGV
#ca_th : min contour area compared to bounding rectangle threshold
markerH = {'min_area':450,'max_area':800,'min_ratio':2,'max_ratio':4.5,'ca_th':0.5}
markerV = {'min_area':200,'max_area':450,'min_ratio':2,'max_ratio':4.5,'ca_th':0.5}

#Navigation parameters
agv_COR = 1.95         #Placement of the center of rotation of the AGV
min_dist_target = 91   #Stopping distance (pixels) when navigating to target (Note: 3.2 pixels per cm)
tol_dist_point = 3     #Stopping distance (pixels) when navigating to a point
ang2t = 7              #Time taken (frames) for rotation of one radian
tol_angle = 0.1        #Tolerance for angle rotation (radians)
T_coords = [250,225]   #Coordinates for T junction (pixels)

#Init variables
agv_coords = [0,440]   #Default position for AGV
rot_angle = 0          #Default rotation angle
motor = [0,0,0,0]      #Bits:[Left motor On, Left motor reverse, Right motor on, Right motor reverse]

#Keep track of current action and target
action = {'mode':'none','timer':0,'dir':1}    #Mode: none,fwd,rot,stop      dir: 0:fwd, 1:rev
nav = {'type':'t','target' : [70,250]}        #Target (t), Waypoint(w), Endpoint(p)

targets = []           #Target list
state = -1             #Default state

#------------Functions-----------#

#Returns angle between two vectors
def angle(v1,v2):
    return np.arctan2(np.cross(v2,v1),np.dot(v2,v1))

#Rounds number to nearest multiple
def nearestmultiple(x,base):
    return base*round(x/base)

#Returns black white image which is a linear sum combination of RGB channels (stored in a)
def linrgb(img,a):
    if(len(a))==3:
        a.append(0)
    img = np.asarray(img,dtype='int16')
    out = a[0]*img[:,:,0]+a[1]*img[:,:,1]+a[2]*img[:,:,2]+a[3]
    out = np.clip(out,a_min=0,a_max=255)
    return np.asarray(out, dtype='uint8' )    

#Check is rectangle is correct for AGV tracking    
def valid_rect(w,h,min_area,max_area,min_ratio,max_ratio,carea,ca_th):
    w,h = int(w),int(h)
    area = w*h
    if area>0 and area >= min_area and area <= max_area and carea>area*ca_th: 
        ratio = max(w/h,h/w)
        if ratio>=min_ratio and ratio<=max_ratio :
            return True
    return False

#Drawing visuals on laptop
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
    cv.circle(output,tuple(T_coords),5,(128,128,128),10)
    cv.drawContours(output,[box_agv],0,(255,255,0),3)
    cv.drawContours(output,[box_grp],0,(255,255,0),3)
    cv.line(output,centre_point,line_point,(0,0,255),1)
    
    #Draw cropped area
    cv.rectangle(output,(tunnel[1],tunnel[0]),(tunnel[3],tunnel[2]),(255,0,0),3)
    cv.rectangle(output,(crop[0],0),(crop[1],width),(255,0,0),3)


#------------Main Loop-----------#

while(cap.isOpened()):
    #Read frame
    ret,frame = cap.read()
    #Quit if no frame
    if frame is None:
        break
    
    #Attempt to connect to arduino
    if connection == False:
        print("Trying to reconnect")
        try:
            connection = telnetlib.Telnet(ip,port,2)
        except:
            connection = False
            print("No Connection")
    
    #Display output
    output = frame+0
    
    #Crop frame
    frame[:,crop[1]:width,:] = 0
    frame[:,0:crop[0],:] = 0
    frame[tunnel[0]:tunnel[2],tunnel[1]:tunnel[3],:] = 0   

    #Read input msg
    prevstate = state + 0
    if connection != False:
        try:
            msg = connection.read_eager()
            if len(msg)>0:
                state = msg[-1]
        except:
            connection = False
  
    #-----Finding the targets-----#
    
    #Isolate green channel and low pass filter
    mask_th_tgt = linrgb(frame,[-2,4,-2])
    mask_th_tgt = cv.medianBlur(mask_th_tgt,5)
    
    #Thresholding and finding countours
    thresh_lower = 150
    cv.imshow('Mask_t',mask_th_tgt)
    _,mask_th_tgt = cv.threshold(mask_th_tgt,thresh_lower,255,cv.THRESH_BINARY)
    contours,_ = cv.findContours(mask_th_tgt,cv.RETR_TREE,cv.CHAIN_APPROX_NONE)
    
    #Store targets in a list
    tgt_count = 0
    targets = []
    for contour in contours:
        (x,y,w,h) = cv.boundingRect(contour)
        if cv.contourArea(contour)>20:
            tgt_count += 1
            cv.rectangle(output,(x,y),(x+w,y+h),(0,255,255),3)
            targets.append([
                int(x+w/2),
                int(y+h/2),
                nearestmultiple( np.linalg.norm([x+w/2-T_coords[0],y+h/2-T_coords[1]]) ,30)
            ])
    
    #Sort by distance, and then by y coordinate
    targets = sorted(targets, key=lambda x:x[1])
    targets = sorted(targets, key=lambda x:x[2])

    
    #-----Finding the AGV------#
    
    #Isolate red and blue channel and low pass filter
    mask_th_agv = linrgb(frame,[2,-4,2,0])
    mask_th_agv = cv.medianBlur(mask_th_agv,5)
    
    #Thresholding and finding countours
    cv.imshow('Mask_agv',mask_th_agv)
    _,mask_th_agv = cv.threshold(mask_th_agv,50,255,cv.THRESH_BINARY)
    contours,_ = cv.findContours(mask_th_agv,cv.RETR_TREE,cv.CHAIN_APPROX_NONE)
    
    #Bounding rectangle
    markers_V = []
    markers_H = []
    for contour in contours:
        #Find bounding rectangle
        rect = cv.minAreaRect(contour)
        (cx,cy), (w,h), rot_angle_rect = rect
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
        agv_coords[0] = int((1-agv_COR)*rect_marker_h[0][0]+agv_COR*rect_marker_v[0][0])
        agv_coords[1] = int((1-agv_COR)*rect_marker_h[0][1]+agv_COR*rect_marker_v[0][1])
    
    #Drawing visuals
    draw_visuals(output,agv_coords,rot_angle)
    
    #-----Navigation------#
    #Init per loop
    motor = [0,0,0,0]
    ArrivedDestination = False
    
    #Set target
    if state == 1:        #State = 1
        if len(targets)>0:
            nav['target'] = targets[0][0:2]
            nav['type'] = 't'
        else:
            #If nothing is seen, go to preset location and grab air
            nav = {'type':'t','target' : [70,250]}
   
    if state == 3:    #State = 3
        if nav['type'] != 'p':
            nav['target'] = [T_coords[0]-90,T_coords[1]]   #Waypoint
            nav['type'] = 'w'
        else:
            nav['target'] = [T_coords[0]+250,T_coords[1]]   #Final point
    
    #Draw line to target
    cv.line(output,tuple(agv_coords),tuple(nav['target']),(0,128,255),1)
    
    #Target angle
    agv_to_target = np.array(nav['target'])-np.array(agv_coords)
    distance_to_target = np.linalg.norm(agv_to_target)
    diff_angle = angle(agv_to_target,[np.cos(rot_angle),np.sin(rot_angle)])
    
    #Reached target
    if distance_to_target < min_dist_target and nav['type'] == 't':
        action['mode'] = 'stop'
        motor = [0,0,0,0]
        ArrivedDestination = True
    
    #Reached final point
    if distance_to_target < tol_dist_point and nav['type'] == 'p':
        action['mode'] = 'stop'
        motor = [0,0,0,0]
        ArrivedDestination = True
        
    #Reached waypoint
    if distance_to_target < tol_dist_point and nav['type'] == 'w':
        action['mode'] = 'stop'
        motor = [0,0,0,0]
        if state == 1:
            nav['type'] = 't'
        elif state == 3:
            nav['type'] = 'p'
            
    #Reset action if timer is zero
    if action['timer'] == 0 and action['mode'] in ['fwd','rot']:
        action['mode'] = 'none'
        action['timer'] = 10
        motor = [0,0,0,0]
    
    #Reset action if angle is too far off
    if abs(diff_angle) > tol_angle and action['mode'] != 'rot':
        action['mode'] = 'none'
        action['timer'] = 10
    
    #If no action taken currently, take new action
    if action['mode'] == 'none' or action['timer'] <= 0:
        if abs(diff_angle) < tol_angle:
            #Move forward to target
            action['mode'] = 'fwd'
            action['timer'] = 10000
            action['dir'] = 0
        else:
            #Rotate to target
            action['mode'] = 'rot'
            action['timer'] = int(abs(diff_angle) * ang2t)
            action['dir'] = (1-np.sign(diff_angle))/2
    
        
        
    #Set motor speeds based on current action
    if action['mode'] == 'fwd':
        motor = [1,action['dir'],1,action['dir']]
    if action['mode'] == 'rot':
        motor = [1,action['dir'],1,1-action['dir']]
    
    #Decrement timer
    action['timer'] -= 1
   
    if state not in [1,3]:
        ArrivedDestination = False
    
    #-----Send signal to arduino------#
    motor = int("".join( [str(int(i)) for i in motor] ),2)
    
    if ArrivedDestination:
        motor += 16   #Set 5th bit to high if arrived at destionation
    
    if state ==1 or state == 3:
        try:
            connection.write(bytes([motor]))
        except:
            connection = False
            print("Message failed to send")
    
    #-----Show output------#
    
    #Show video output
    cv.imshow('output',output)
    
    #Press q to quit
    if cv.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv.destroyAllWindows()
        break



#-----End sequence------#

cap.release()
cv.destroyAllWindows()
