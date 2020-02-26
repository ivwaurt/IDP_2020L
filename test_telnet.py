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
        t = t+1
        connection.write(("Testing "+str(t)+"\n").encode('ascii'))
        time.sleep(1)