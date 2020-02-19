import numpy as np
import cv2

cap = cv2.VideoCapture("testvideo5.mp4")

# Get the Default resolutions
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))

# Define the codec and filename.
fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
out = cv2.VideoWriter('output.avi',fourcc, 30, (frame_width,frame_height))

while(cap.isOpened()):
    ret, frame = cap.read()
    if ret==True:
        # write the  frame
        out.write(frame)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cv2.imshow('frame',hsv[:,:,1])
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

# Release everything if job is finished
cap.release()
out.release()
cv2.destroyAllWindows()