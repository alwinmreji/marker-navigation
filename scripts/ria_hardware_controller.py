#! /usr/bin/env python
import rospy
from std_msgs.msg import String
import serial
arduino = serial.Serial('/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',9600, timeout=.1)
value = {'EAR_ON':'3','EAR_OFF':'4','SKRT_ON':'1','SKRT_OFF':'2','EAR_FD':'7','EAR_BLINK':'8','SKRT_FD':'5','SKRT_BLINK':'6','BAT':'9'}


def cmdCallback(msg):
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



def listener():
    rospy.init_node('ria_hardware_controller',anonymous=True)
    rospy.Subscriber('/ria/hardware/cmd',String,cmdCallback)
    pub = rospy.Publisher('/ria/hardware/status',String, queue_size=1)
    rate = rospy.Rate(0.02)
    while not rospy.is_shutdown():
        arduino.write(value['BAT'].encode())
        arduino.flushInput()
        res = arduino.readline()
        while not res:
            res = arduino.readline()
            pass
        pub.publish(res)
        rate.sleep()

if __name__=='__main__':
    listener()
