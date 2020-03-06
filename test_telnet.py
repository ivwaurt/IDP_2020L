import telnetlib
import time



ip = "192.168.43.224"
port = 23
connection = False
msg = -1

while 1:
    print(connection)
    
    
    #Try to connect
    if connection == False:
        print("Trying to reconnect")
        try:
            connection = telnetlib.Telnet(ip,port,2)
        except:
            connection = False
            print("No Connection")
            
            
    #Try to read message
    if connection != False:
        try:
            msg = connection.read_eager()
        except:
            connection = False
    print(msg)
    
    
    #Try to write message
    motor = 14
    try:
        connection.write(bytes([motor]))
    except:
        connection = False
        print("Message failed to send")
    
    #Simulate waiting for next frame
    time.sleep(0.3)