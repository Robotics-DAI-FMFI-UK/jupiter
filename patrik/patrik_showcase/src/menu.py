#! /usr/bin/env python
import os
import subprocess

print('Start');
stream = os.popen('ps -u  mustar| grep "python" | grep -o "^ *[0-9]*"')
default = stream.read();
default = default.split();
default2=[];
for i in default:
	 default2.append(int(i));
default = default2;
print(default2);


def movenexttowall():
    stoptask()
    comand1 = subprocess.Popen(['python' , '/home/mustar/catkin_ws/src/patrik_showcase/src/Moving_next_wall.py'])
    stream = os.popen('ps -u  mustar| grep "python" | grep -o "^ *[0-9]*"')
    movenexttowall = stream.read()
    print(movenexttowall);

def forward():
    stoptask()
    comand1 = subprocess.Popen(['python' , '/home/mustar/catkin_ws/src/patrik_showcase/src/forward.py'])
    stream = os.popen('ps -u  mustar| grep "python" | grep -o "^ *[0-9]*"')
    movenexttowall = stream.read()
    print(movenexttowall);

def rotate(x):
    stoptask()
    comand1 = subprocess.Popen(['python' , '/home/mustar/catkin_ws/src/patrik_showcase/src/rotate_'+ x +'.py'])
    stream = os.popen('ps -u  mustar| grep "python" | grep -o "^ *[0-9]*"')
    movenexttowall = stream.read()
    print(movenexttowall);

def stoptask():
    stream = os.popen('ps -u  mustar| grep "python" | grep -o "^ *[0-9]*"')
    stoped = stream.read()
    stoped = stoped.split();
    for i in stoped:
	if (int(i) not in default):
		print(i);
		os.system('kill -TERM ' + i);
    #Moving_next_wall.GoToWall();



####### arm controling-----------
import shlex
robotarm = {
'waist': 0.0,
'shoulder':0.0,
'elbow': 0.0,
'wrist':0.0,
'hand': 0.0
}
def armwrite(name,x):
	robotarm[name]+= x;
	text = 'rostopic pub -1 /' + name + '_controller/command std_msgs/Float64 -- '+str(robotarm[name]);
	args = shlex.split(text)
	print(args);
	comand1 = subprocess.Popen(args);
'''
class armcontrolmove:
	def __init__(self,name,dif):
		self.name=name;
		self.dif=dif;
	def armwrite(self, robotarm):
		robotarm[self.name]+= self.dif;
		text = 'rostopic pub -1 /' + self.name + '_controller/command std_msgs/Float64 -- '+ str(robotarm[self.name]);
		stream = os.popen(text);
'''
#--------------------------------
import Tkinter as tk
import tkFont;

root = tk.Tk()
root.geometry("2000x1000")

helv30 = tkFont.Font(family="Helvetica",size=32,weight="bold")
helv20 = tkFont.Font(family="Helvetica",size=30,weight="bold")
helv10 = tkFont.Font(family="Helvetica",size=20,weight="bold")
frame = tk.Frame(root)
frame.pack()


button = tk.Button(frame, 
                   text="QUIT", 
                   fg="red",
                   command=quit,
		   font=helv20,
		   height = 2);
button.grid(sticky="news",row = 0 , column =0);

# arm
from functools import partial

arm_buttons = [None]*10;
j=0;
rownum = 1;
for i in robotarm.keys():
    rownum +=1;
    j+=2;
    partarm = partial(armwrite,i,0.2);
    arm_buttons[j-2]=tk.Button(frame,
                   text=i,font=helv20,
                   command=partarm,width =20, height = 2,
	           bg='#90ee90', borderwidth=6)
    arm_buttons[j-2].grid(sticky="news",row = rownum, column = 0)
    
    partarm = partial(armwrite,i,-0.2);
    arm_buttons[j-1]=tk.Button(frame,
                   text='-'+i,font=helv20,
                   command=partarm,width = 20,
                   bg='#FFCCCB', borderwidth=6)
    arm_buttons[j-1].grid(sticky="news",row = rownum, column = 1)
    print(partarm)
#arm

wall= tk.Button(frame,
                   text="follow the wall",font=helv10,
                   command =movenexttowall,bg='#add8e6', borderwidth=6) 
wall.grid(sticky="news",row = 5 , column =3, columnspan = 2);

dopredu= tk.Button(frame,
                   text="forward",font=helv20,
                   command =forward, width =40, bg='#add8e6', borderwidth=6) 
dopredu.grid(sticky="news",row = 1 , column =3,rowspan = 3, columnspan=2);
right= tk.Button(frame,
                   text="right",font=helv20,
                   command =lambda: rotate('l'), bg='#add8e6', borderwidth=6) 
right.grid(sticky="news",row = 4 , column =3);
left= tk.Button(frame,
                   text="left",font=helv20,
                   command =lambda: rotate('r'),bg='#add8e6', borderwidth=6) 
left.grid(sticky="news",row = 4 , column =4);

button2 = tk.Button(frame, 
                   text="STOP", font=helv30,
                   fg="white",
                   command=stoptask, bg="#8b0000", borderwidth=6)
button2.grid(sticky="news", row = 6 , column =3, columnspan = 2,rowspan = 7);
root.mainloop()



