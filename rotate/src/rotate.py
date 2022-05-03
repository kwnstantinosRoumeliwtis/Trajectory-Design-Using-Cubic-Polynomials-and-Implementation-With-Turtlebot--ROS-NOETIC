#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist,Point
import sys
import time
import math
import numpy as np
import matplotlib.pyplot as plt
i=0.0;
x=0.0
y=0.0
theta=0.0
flag=0
start=0.0
end=0.0
k=0
lx=0
ly=0
st=0
lt=0
et=0
time1=[]
xo=[]
yo=[]
posx1=[]
posy1=[]
posx2=[]
posy2=[]
fullx=[]
fully=[]
vel=[]
vel1=[]
vel2=[]
veld=[]



def nOdometry(msg):
	global x
	global y
	global theta
	x=msg.pose.pose.position.x
	y=msg.pose.pose.position.y	
	rot_q=msg.pose.pose.orientation
	(roll,pitch,theta)=euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
     
def stop(move):
	move.linear.x=0
	move.linear.y=0
	move.angular.z=0

def delay():
	start=time.time()
	end=0			
	while end-start<3.0:					
		end=time.time()
     
def calc_ai(dt,dist):
	a2=(3.0/(dt**2))*(dist)
	a3=-(2.0/(dt**3))*(dist)
	return a2,a3
	
def fix_final_angle(rate,pub,move):
	print("wait for final angle...")
	global theta
	while theta<1.5824:
		move.angular.z=0.1
		pub.publish(move)
		rate.sleep()
	move.angular.z=0.0
	pub.publish(move)
	rate.sleep()   

rospy.init_node("trajectory")
sub=rospy.Subscriber("/odom",Odometry,nOdometry)

pub=rospy.Publisher('/cmd_vel', Twist,queue_size=1)
move=Twist()

r=rospy.Rate(1000)
goal1=Point()
goal2=Point()
goal1.x=16.0
goal1.y=8.0 
goal2.x=16.0
goal2.y=-8.0
dt1=130.0
dt2=130.0
xfaa2,xfaa3=calc_ai(dt1,0.0)
yfaa2,yfaa3=calc_ai(dt1,-16.0)
xa2,xa3=calc_ai(dt1,16.0)
ya2,ya3=calc_ai(dt1,8.0)

while not rospy.is_shutdown() : 
	if flag==0:
		if(et-st==0):
			vel.append(0)
		else:
			vel.append(math.sqrt(((epx-startx)/(et-st))**2+((epy-starty)/(et-st))**2)/100)
		time1.append(i)
		inc_x=goal1.x-x
		inc_y=goal1.y-y
		startx=x
		starty=y
		st=time.time()
		angle=np.arctan2(inc_y,inc_x)
		move.linear.x=xa2*2.0*i+3.0*xa3*i**2
		move.linear.y=ya2*2.0*i+3.0*ya3*i**2
		xo.append(x)
		yo.append(y)		
		if i>0 :	
			move.angular.z=((angle-theta)/i)
		print(x,y,i,math.sqrt(move.linear.x**2+move.linear.y**2))	
		i=i+0.001
		if i>=dt1:		        
			stop(move)
			delay()		
			flag=1
			o=i
			i=0.0										
	else:                
		if k==0:
			et=0			
			xfa2,xfa3=calc_ai(dt2,16.0)
			yfa2,yfa3=calc_ai(dt2,0.0)
			k=1
		if(et-st==0):
	      		vel.append(0)
		else:
			vel.append(math.sqrt(((epx-startx)/(et-st))**2+((epy-starty)/(et-st))**2)/100)	
		inc_x=goal2.x-x
		inc_y=goal2.y-y
		startx=x
		starty=y
		st=time.time()
		angle=np.arctan2(inc_y,inc_x)
		move.linear.x=xfa2*2.0*i+3.0*xfa3*i**2
		move.linear.y=yfa2*2.0*i+3.0*yfa3*i**2
		xo.append(x)
		yo.append(y)
		print(x,y,i,math.sqrt(move.linear.x**2+move.linear.y**2))			
		if i>0:			
			move.angular.z=((angle-theta)/i)
		time1.append(o)	
		i=i+0.001
		o=o+0.001
		if  i>=dt2:
			stop(move)
			pub.publish(move)
			r.sleep()
			print("theta this moments is "+str(theta))
			break
				
	pub.publish(move)
	r.sleep()
	et=time.time()
	epx=x
	epy=y
	
fix_final_angle(r,pub,move)
print("3 seconds more...")
delay()	
print(theta)
print("one moment for graphs...")

for i in range(130000):
  posx1.append(xa2*time1[i]**2+xa3*time1[i]**3)
  posy1.append(ya2*time1[i]**2+ya3*time1[i]**3)
  posx2.append(16+xfaa2*time1[i]**2+xfaa3*time1[i]**3)
  posy2.append(8+yfaa2*time1[i]**2+yfaa3*time1[i]**3)

fullx=posx1+posx2
fully=posy1+posy2

fig, (ax1, ax2) = plt.subplots(1, 2, constrained_layout=True, sharey=True)
ax1.plot(time1,xo,'b',label="position x")
ax1.plot(time1,yo,'r',label="position y")
ax1.set_title(' real trajectory',fontsize=20)
ax1.set_xlabel('time (s)')
ax1.set_ylabel('positions x&y')

ax2.plot(time1,fullx,'b',label="position x")
ax2.plot(time1,fully,'r',label="position y")
ax2.set_title('desired trajectory',fontsize=20)
ax2.set_xlabel('time (s)')
ax2.set_ylabel('positions x&y')

fig.suptitle('x:blue line     y:red line', fontsize=10)

fig, (ax3, ax4) = plt.subplots(1, 2, constrained_layout=True, sharey=True)
ax3.plot(xo,yo,'k',label="real route")
ax3.set_title(' real route',fontsize=20)
ax3.set_xlabel('x position')
ax3.set_ylabel('y position')

ax4.plot(fullx,fully,'k',label="desired route")
ax4.set_title('desired route',fontsize=20)
ax4.set_xlabel('x position')
ax4.set_ylabel('y position')

xfaa2,xfaa3=calc_ai(dt1,16.0)
yfaa2,yfaa3=calc_ai(dt1,0.0)

i=0
while i<130:
	x1=xa2*2.0*i+3.0*xa3*i**2
	y1=ya2*2.0*i+3.0*ya3*i**2
	x2=xfaa2*2.0*i+3.0*xfaa3*i**2
	y2=yfaa2*2.0*i+3.0*yfaa3*i**2
	vel1.append(math.sqrt(x1**2+y1**2))
	vel2.append(math.sqrt(x2**2+y2**2))
	i+=0.001
	
veld=vel1+vel2 

fig, (ax5, ax6) = plt.subplots(1, 2, constrained_layout=True, sharey=True)
ax5.plot(time1,vel,'k',label="real velocity")
ax5.set_title(' real velocity',fontsize=20)
ax5.set_xlabel('time (s)')
ax5.set_ylabel('velocity')

ax6.plot(time1,veld,'k',label="desired velocity")
ax6.set_title('desired velocity',fontsize=20)
ax6.set_xlabel('time (s)')
ax6.set_ylabel('velocity')

plt.show()	






