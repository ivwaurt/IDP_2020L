
state = -1
msg = "OOHELLO"
if len(msg)>5: 
    if msg[-6].isdigit():
        state = int(msg[-6])
print(state)