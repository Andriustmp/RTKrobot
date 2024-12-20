
"""
 RTK robot mower V 1.1
 TCP client to rtklib software
 2024 05 23
 author: A. Besusparis
"""

import json
import serial
import time
import socket
import threading 

import datetime

host = '192.168.25.12'
port = 5555

RTKdata= ["0", "0", "0","0", "0", "0","0", "0", "0","0", "0", "0","0", "0", "0","0","0","0","0","0",]

class TCPconnect(threading.Thread):
    def __init__(self, shared, shared2, shared3, sock=None,):

        self.shared = shared
        self.shared2 = shared2
        self.shared3 = shared3
        if sock is None:
            self.sock = socket.socket(
                            socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock

    def connect(self, host, port):
        try:
            self.sock.connect((host, port))
            print('RTKrcv TCP Successful Connection')
        except:
            print('RTKrcv TCP Connection Failed')

    def readlines(self):
        data = self.sock.recv(1024).decode()
        x = data.split()
        RTKdata=x;
        self.decode(RTKdata)
        #print(RTKdata)
        #print(data.decode())

    def run(self):
        self.connect(host,port)
        while True:
           self.readlines()

    def decode(self, RTKdata):
        self.shared.PacketCnt+=1  
        self.shared.GPSTdate=RTKdata[0]
        self.shared.GPSTtime=RTKdata[1] 
        self.shared.lat=float(RTKdata[2])+self.shared3.LatOfset 
        self.shared.long=float(RTKdata[3])+self.shared3.LongOfset 
        self.shared.height=float(RTKdata[4]) 
        self.shared.Q=int(RTKdata[5]) 
        self.shared.ns=int(RTKdata[6]) 
        self.shared.sdn=RTKdata[7] 
        self.shared.sde=RTKdata[8] 
        self.shared.sdu=RTKdata[9] 
        self.shared.sdne=RTKdata[10] 
        self.shared.sdeu=RTKdata[11] 
        self.shared.sdun=RTKdata[12] 
        self.shared.age=RTKdata[13] 
        self.shared.ratio=float(RTKdata[14]) 

        #self.shared2.AddRecord(RTKdata[2],RTKdata[3],RTKdata[5])
               