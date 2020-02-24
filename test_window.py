from tkinter import *
import time
root=Tk()

variable=StringVar()

def update_label():
    i=0
    while 1:
        i=i+1
        variable.set(str(i))
        root.update()

your_label=Label(root,textvariable=variable)
your_label.pack()
start_button=Button(root,text="start",command=update_label)
start_button.pack()
root.mainloop()