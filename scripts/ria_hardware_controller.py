#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from math import pi
from std_srvs.srv import SetBool, Trigger
import serial

arduino = serial.Serial('/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',9600, timeout=.1)
value = {'EAR_ON':'3','EAR_OFF':'4','SKRT_ON':'1','SKRT_OFF':'2','EAR_FD':'7','EAR_BLINK':'8','SKRT_FD':'5','SKRT_BLINK':'6','BAT':'9'}
nav_pub = rospy.Publisher('/ria/odom/local/goal', Twist, queue_size=1)
stop = rospy.ServiceProxy('/ria/odom/goal/stop',SetBool)
dock = rospy.ServiceProxy('ria/odom/marker/search',SetBool)
goal_reset = rospy.ServiceProxy('ria/odom/goal/reset', Trigger)
odom_reset = rospy.ServiceProxy('ria/odom/reset', Trigger)

def cmdCallback(msg):
    global value, arduino
    cmd = msg.data.split(" ")[0]
    try:
        param = int(msg.data.split(" ")[1])

        while param:
            rospy.loginfo("Writing Command:"+cmd)
            arduino.write(value[cmd].encode())
            arduino.flushInput()
            rospy.sleep(5)
            #res = arduino.readline()

            param-=1
    except IndexError:
        rospy.loginfo("Writing Command:"+cmd)
        arduino.write(value[cmd].encode())

def navCallback(msg):
    global nav_pub, value, stop, dock, goal_reset, odom_reset
    goal_reset()
    odom_reset()
    cmd = msg.data.split(" ")[0]
    goal = Twist()
    try:
        val = float(msg.data.split(" ")[1])
        if (cmd == 'FWD'):
            goal.linear.x = val
        elif (cmd == 'BCK'):
            goal.angular.z = pi
            goal.linear.x = val
        elif (cmd == 'RGT'):
            goal.angular.z = pi/2.0
        elif (cmd == 'LFT'):
            goal.angular.z = -1*(pi/2.0)
        nav_pub.publish(goal)
    except:
        if (cmd == 'STOP'):
            result = stop(True)
            rospy.loginfo("Stopped Goal: {}".format(result))
        elif (cmd == 'CHARG'):
            result = dock(True)
            rospy.loginfo("Marker Search Started: {}".format(result))
        elif (cmd == 'RGT'):
            goal.angular.z = pi/2.0
            nav_pub.publish(goal)
        elif (cmd == 'LFT'):
            goal.angular.z = -1*(pi/2.0)
            nav_pub.publish(goal)

def listener():
    global stop
    rospy.init_node('ria_hardware_controller',anonymous=True)
    rospy.Subscriber('/ria/hardware/cmd',String,cmdCallback)
    rospy.Subscriber('/ria/nav/cmd',String,navCallback)
    pub = rospy.Publisher('/ria/hardware/status',String, queue_size=1)
    rate = rospy.Rate(0.02)
    while not rospy.is_shutdown():
        #arduino.write(value['BAT'].encode())
        arduino.flushInput()
        res = arduino.readline()
        while not res:
            res = arduino.readline()
            pass
        try:
            if(res.strip()=="9" or res.strip()=="10"):
                result=stop(True)
                rospy.loginfo("Stopped Goal:{}".format(result))
        except rospy.ServiceException as e:
            rospy.logerr(e)
        pub.publish(res.strip())
        #rate.sleep()

if __name__=='__main__':
    listener()
