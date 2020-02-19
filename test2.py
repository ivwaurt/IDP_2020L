import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt

#Import video
cap = cv.VideoCapture("testvideo5.mp4")
#cap = cv.VideoCapture(1)
width = int(cap.get(3))
height =int(cap.get(4))
print("Vid dimentions: ",width,"x",height)
crop_points = [[0,0],[int(height),int(width*0.65)]]


fourcc = cv.VideoWriter_fourcc(*'DIVX')
out = cv.VideoWriter('output.avi',fourcc, 20, (width,height))

def isofilter(img,index,bias):
    img = np.asarray(img,dtype='int16')
    out = 2*(3*img[:,:,index]-np.sum(img,axis=2))+bias
    out = np.clip(out,a_min=0,a_max=255)
    return np.asarray(out, dtype='uint8' )



circle_centers = []
while(cap.isOpened()):
    ret,frame = cap.read()
    #Quit if no frame
    if frame is None:
        break
    
    #Crop frame
    frame = frame[crop_points[0][0]+0:crop_points[1][0]+0, crop_points[0][1]+0:crop_points[1][1]+0]
    output = frame
    

    
    #Convert to blue and blur
    frame = isofilter(frame,2,-100)
    #frame = cv.cvtColor(frame,cv.COLOR_RGB2GRAY)
    frame = cv.medianBlur(frame,21)
    
    cv.imshow('frame',frame)
    #Find circles
    circles = cv.HoughCircles(
        image=frame,
        method=cv.HOUGH_GRADIENT,
        dp=1,
        minDist=30,
        param1=80,
        param2=30,
        minRadius=15,
        maxRadius=60
    )
    
    if circles is not None:
        detected_circles = np.uint16(np.around(circles))
        for (x,y,r) in detected_circles[0,::-1]:
            cv.circle(output,(x,y),r,(0,255,0),2)
        circle_centers.append([x,y])
    
    #Show output
    cv.imshow('output',output)
    out.write(frame)
    
    #Press q to quit
    if cv.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv.destroyAllWindows()
        break

out.release()
circle_x = np.array(circle_centers)[:,0]
circle_y = np.array(circle_centers)[:,1]
#x = np.convolve(x,np.ones(5),'valid')
plt.plot(circle_x,circle_y)
plt.show()
	