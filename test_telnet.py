import telnetlib
import time



ip = "192.168.4.1"
port = 23
conn_tout = 5
connection = telnetlib.Telnet(ip,port,conn_tout)

t=0
while 1:
    text = connection.read_until("HELLO".encode('ascii'),1)
    print(text)
    if text != '':
        v = input("control motor")
        if v=='0' :
            byte = 0 #0000
        elif v=='1' : 
            byte = 10 #1010
        elif v=='2' :
            byte = 15 #1111
        byte = bytes([byte])
        print(byte)
        connection.write(byte)
        t = t+1
        #connection.write(("Testing "+str(t)+"\n").encode('ascii'))
        time.sleep(1)