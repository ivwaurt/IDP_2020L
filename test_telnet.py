import telnetlib
import time



ip = "192.168.43.156"
port = 23
conn_tout = 5
connection = telnetlib.Telnet(ip,port,conn_tout)

t = 0
while t<1000:
    connection.write(("Testing "+str(t)+"\n").encode('ascii'))
    t = t+1
    time.sleep(1)