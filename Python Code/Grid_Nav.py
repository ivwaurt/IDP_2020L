#Init variables
agv_coords = [0,0]
rot_angle = 0
action = {'mode':'none','timer':0,'dir':1}   #Mode: none,fwd,rot,stop      dir: 0:fwd, 1:rev
motor = [0,0,0,0]   #:[Left motor On, Left motor reverse, Right motor on, Right motor reverse]
target_coords = [0,0]
targets = []


#-----Navigation------#
    #Target angle
    #Set condition for grid approach

    if abs(np.array(target_coords)[1]-np.array(agv_coords)[1])>tol:
        agv_to_target = np.array(agv_coords[0],target_coords[1])
        diff_angle = angle(agv_to_target,[np.cos(rot_angle),np.sin(rot_angle)])
        agv_to_target_real = np.array(target_coords)-np.array(agv_coords)
        distance_to_target = np.linalg.norm(agv_to_target_real)
        if distance_to_target_real <= min_dist:    #add preset value(length of gripper to wheel)
            diff_angle = angle(agv_to_target_real,[np.cos(rot_angle),np.sin(rot_angle)])
            #Break events
            if abs(diff_angle) < tol_angle:
                action['mode'] = 'stop'
                motor = [0,0,0,0]
                #Done .... What next?

    else:
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
    
    #If no action, take new action
    if action['mode'] == 'none' and action['timer'] = 0:
        #Insert pathfinding algorithm here
        if abs(diff_angle) < tol_angle:
            action['mode'] = 'fwd'
            action['timer'] = -1         #-1 = do forever
        else:
            #Rotate to target
            action['mode'] = 'rot'
            action['timer'] = abs(diff_angle) * ang2t
            action['dir'] = (1-np.sign(diff_angle))/2
    
    #Set velocity for forward and rotation
    if action['mode'] == 'fwd':
        motor = [1,action['dir'],1,action['dir']/2]
    if action['mode'] == 'rot':
        motor = [1,action['dir'],1,1-action['dir']]
    
    #Decrement timer
    action['timer'] -= 1
    