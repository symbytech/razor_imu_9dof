#!/usr/bin/env python

# Copyright (c) 2012, Tang Tiong Yew
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import serial
import serial.tools.list_ports
import string
import math
import sys

#from time import time
from razor_imu_9dof.msg import IMUeuler



degrees2rad = math.pi/180.0



rospy.init_node("razor_node")
pub_euler = rospy.Publisher('imu_euler', IMUeuler, queue_size=1)

imueulerMsg = IMUeuler()

# default_port='/dev/IMU'
# port = rospy.get_param('~port', default_port)
# baud = rospy.get_param('~baud')


# Check your COM port and baud rate
rospy.loginfo("Opening port")
try:
    for port in serial.tools.list_ports.comports():
        if port.serial_number == "C719D0C05150334C414A2020FF011C26":
            ser = serial.Serial(port.device, 9600)
except serial.serialutil.SerialException:
    rospy.logerr("IMU not found at port "+port + ". Did you specify the correct port in the launch file?")
    #exit
    sys.exit(0)

roll=0
pitch=0
yaw=0
seq=0
turn_count=0
rospy.loginfo("Giving the razor IMU board 5 seconds to boot...")
rospy.sleep(5) # Sleep for 5 seconds to wait for the board to boot

####NEWBOARD####

#we can only toggle settingonand off, so we must assume someone trustworthy has
#set the IMU correctly


rospy.loginfo("Flushing first 200 IMU entries...")
for x in range(0, 200):
    line = ser.readline()
rospy.loginfo("Publishing IMU data...")

rate = rospy.Rate(1) #1Hz
yaw=0
prev_yaw=0
while not rospy.is_shutdown():
    #print("imu node running")
    line = ser.readline()
    #print(line)
    #line = line.replace("#YPRAG=","")   # Delete "#YPRAG="
    words = string.split(line,",")    # Fields split
    if len(words) > 2: #we have pitch roll and yaw
        prev_yaw = yaw
        yaw = float(words[2])
        pitch = float(words[0])
        roll = float(words[1])

        #we will take the ooportunity to count rotations
        if yaw < 10 and prev_yaw > 300:
            turn_count = turn_count + 1
        elif yaw > 300 and prev_yaw < 10:
            turn_count = turn_count - 1
        print(yaw, prev_yaw)

    # Publish message
    imueulerMsg = IMUeuler()
    imueulerMsg.angle.x = roll
    imueulerMsg.angle.y = pitch
    imueulerMsg.angle.z = yaw
    imueulerMsg.turncount = turn_count
    imueulerMsg.header.stamp= rospy.Time.now()
    imueulerMsg.header.frame_id = 'base_imu_link'
    imueulerMsg.header.seq = seq
    #print(imueulerMsg)
    pub_euler.publish(imueulerMsg)
    seq = seq + 1
 
    rate.sleep()
        
ser.close
#f.close
