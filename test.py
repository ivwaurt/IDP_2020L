

v = input("control motor")
if v=='0' :
    byte = 0 #0000
elif v=='1' : 
    byte = 10 #1010
elif v=='2' :
   byte = 15 #1111
byte = bytes([byte])


print(("Testing \n").encode('ascii'))
print(byte)