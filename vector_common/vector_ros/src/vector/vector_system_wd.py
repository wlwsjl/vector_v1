"""--------------------------------------------------------------------
COPYRIGHT 2016 Stanley Innovation Inc.

Software License Agreement:

The software supplied herewith by Stanley Innovation Inc. (the "Company") 
for its licensed SI Vector Platform is intended and supplied to you, 
the Company's customer, for use solely and exclusively with Stanley Innovation 
products. The software is owned by the Company and/or its supplier, and is 
protected under applicable copyright laws.  All rights are reserved. Any use in 
violation of the foregoing restrictions may subject the user to criminal 
sanctions under applicable laws, as well as to civil liability for the 
breach of the terms and conditions of this license. The Company may 
immediately terminate this Agreement upon your use of the software with 
any products that are not Stanley Innovation products.

The software was written using Python programming language.  Your use 
of the software is therefore subject to the terms and conditions of the 
OSI- approved open source license viewable at http://www.python.org/.  
You are solely responsible for ensuring your compliance with the Python 
open source license.

You shall indemnify, defend and hold the Company harmless from any claims, 
demands, liabilities or expenses, including reasonable attorneys fees, incurred 
by the Company as a result of any claim or proceeding against the Company 
arising out of or based upon: 

(i) The combination, operation or use of the software by you with any hardware, 
    products, programs or data not supplied or approved in writing by the Company, 
    if such claim or proceeding would have been avoided but for such combination, 
    operation or use.
 
(ii) The modification of the software by or on behalf of you 

(iii) Your use of the software.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 
 \file   vector_system_wd.py

 \brief  This is the vector system watchdod which monitors signals
         from the embedded power system to safely shutdown the PC
         upon embedded powerdown

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
import socket
import sys
import rospy
import os
from std_msgs.msg import Bool
from utils import m32
import socket
import re

class VectorWatchdog:
    def __init__(self):
        """
        Initialize the UDP connection
        """
        self._remote_shutdown_msg = Bool()
        self._remote_shutdown_msg.data = True
        if not (None == re.match('vector1',socket.gethostname())):
            self.remote_sub = rospy.Subscriber('vector1/shutdown_pc',Bool,self._shutdown_cb)
            self.remote_pub = rospy.Publisher('vector2/shutdown_pc',Bool,queue_size=1,latch=True)
        else:
            self.remote_sub = rospy.Subscriber('vector2/shutdown_pc',Bool,self._shutdown_cb)
            self.remote_pub = rospy.Publisher('vector1/shutdown_pc',Bool,queue_size=1,latch=True)
            
        self._continue = True     
        self.conn = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.conn.setblocking(False)
        self.conn.bind(('',6234))
        
    def _shutdown_cb(self,msg):
        rospy.logerr("Platform signaled shutdown, need to shutdown the onboard PC")
        self.Close()
        os.system("sudo shutdown now -h")
        sys.exit(0)
            
    def Receive(self):
        """
        Try receiving the data up to a maximum size. If it fails
        empty the data
        """
        try:
            data = self.conn.recv(4)
        except:
            data = []            
            
        if (len(data) == 4):
        
            rx_dat = [ord(i) for i in data]
            shutdwn_cmd = m32(rx_dat)
            if (0x8756BAEB == shutdwn_cmd):
                self.remote_pub.publish(self._remote_shutdown_msg)
                rospy.sleep(1.0)
                self._shutdown_cb(True)
    def Close(self):
        self.conn.close()       
        
    
    
    
    
    
