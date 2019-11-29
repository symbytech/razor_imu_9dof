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
import string
import math
import sys

#from time import time
from razor_imu_9dof.msg import IMUeuler
from dynamic_reconfigure.server import Server
from razor_imu_9dof.cfg import imuConfig
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

degrees2rad = math.pi/180.0

# Callback for dynamic reconfigure requests
def reconfig_callback(config, level):
    global imu_yaw_calibration
    rospy.loginfo("""Reconfigure request for yaw_calibration: %d""" %(config['yaw_calibration']))
    #if imu_yaw_calibration != config('yaw_calibration'):
    imu_yaw_calibration = config['yaw_calibration']
    rospy.loginfo("Set imu_yaw_calibration to %d" % (imu_yaw_calibration))
    return config

rospy.init_node("razor_node")
pub_euler = rospy.Publisher('imu_euler', IMUeuler, queue_size=1)

imueulerMsg = IMUeuler()

default_port='/dev/ttyUSB0'
port = rospy.get_param('~port', default_port)
baud = rospy.get_param('~baud')


# Check your COM port and baud rate
rospy.loginfo("Opening %s...", port)
try:
    ser = serial.Serial(port=port, baudrate=baud, timeout=1)
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


while not rospy.is_shutdown():
    print("imu node running")
    line = ser.readline()
    #line = line.replace("#YPRAG=","")   # Delete "#YPRAG="
    words = string.split(line,",")    # Fields split
    if len(words) > 2: #we have pitch roll and yaw
        #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
        yaw = float(words[2])

        #yaw accumulates, we need to reset it
        #we will take the ooportunity to count rotations

        if yaw_deg > 180.0:
            yaw_deg = yaw_deg - 360.0
            turn_count = turn_count - 1
        if yaw_deg < -180.0:
            yaw_deg = yaw_deg + 360.0
            turn_count = turn_count + 1
        yaw = yaw_deg*degrees2rad
        #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
        pitch = float(words[0])*degrees2rad
        roll = float(words[1])*degrees2rad

    # Publish message
    imueulerMsg = IMUeuler()
    imueulerMsg.angle.x = roll
    imueulerMsg.angle.y = pitch
    imueulerMsg.angle.z = yaw
    imueulerMsg.turn_count = turn_count
    imueulerMsg.header.stamp= rospy.Time.now()
    imueulerMsg.header.frame_id = 'base_imu_link'
    imueulerMsg.header.seq = seq
    pub_euler.publish(imueulerMsg)
    seq = seq + 1
 
        
ser.close
#f.close
