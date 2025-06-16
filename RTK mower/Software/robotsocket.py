"""
 RTK robot mower V 1.4
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

host = 'xx.xx.xx.xx'  # Robot Mower IP
port = 5555

RTKdata= ["0", "0", "0","0", "0", "0","0", "0", "0","0", "0", "0","0", "0", "0","0","0","0","0","0",]

class TCPconnect(threading.Thread):
    def __init__(self, shared, shared2, shared3, sock=None,):

        self.shared  = shared
        self.shared2 = shared2
        self.shared3 = shared3
        if sock is None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.settimeout(5)                             
        else:
            self.sock = sock

    def connect(self, host, port):
        try:
            self.sock.connect((host, port))
            print('RTKrcv TCP Successful Connection')
        except:
            print('RTKrcv TCP Connection Failed')

    def readlines(self):    
        data=False 
        try:
            data = self.sock.recv(1024).decode()
            
        except socket.error as e:
            print(f"Error during data exchange: {e}")
                 
        if  data: 
            x = data.split()
            RTKdata=x;
            self.decode(RTKdata)
        else:
            self.sock.close()
            time.sleep(2)
            #print("TCP socket: no data from RTKrcv, reconecting ...")
            self.shared.ValidConn=False
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            try:
                 self.sock.connect((host, port))
                 print('TCP socket: reconnected')
            except:
                 print('TCP socket: reconnect failed')

    def run(self):
        self.connect(host,port)   
        while True:
           self.readlines()

    def decode(self, RTKdata):
        self.shared.ValidConn=True
        self.shared.PacketCnt+=1  
        self.shared.GPSTdate=RTKdata[0]
        self.shared.GPSTtime=RTKdata[1] 
        self.shared.lat=float(RTKdata[2])
        self.shared.long=float(RTKdata[3])
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