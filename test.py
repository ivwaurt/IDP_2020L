def nearestmultiple(x,base):
    return base*round(x/base)
    
for i in range(500):
    print(nearestmultiple(i,15))