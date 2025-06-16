"""
 RTK robot mower V 1.4
 RTK robot  main
 2024 05 23
 author: A. Besusparis
"""

import os
import datetime
import geojson
import threading
import serial
import time
import socket
import csv
import time

from robotserial  import rserial
from robotwww     import robotweb
from robotsocket  import TCPconnect
from navigation   import Robotmove

from sshkeyboard import listen_keyboard

timestr = time.strftime("%Y%m%d-%H%M%S")

# Serial data to MCU
class TXData:
     def __init__ (self ):
         self.Transmit       = 1   # first time all motors off
         self.LmotorFR       = 'F'
         self.Lspeed         = 0x00
         self.RmotorFR       = 'F'
         self.Rspeed         = 0x00
         self.Cutter_RunStop = 0x00   

# Serial data from MCU  ( 200 ms TX rate )
class RXData:
     def __init__ (self ):
         self.Recived    = 0      # Set to 1 after packet is recived
         self.PacketCnt  = 0     
         self.Estop      = 0      # Robot bumper or E-Stop button active ( stop from MCU)
         self.Ebutton    = 0
         self.Esensor    = 0
         self.BatVoltage = 0
         self.BatCurrent = 0
         self.Direction  = 0      # 0-360 (deg)
         self.LWheelCnt  = 0      # start from ~5000
         self.RWheelCnt  = 0      # start from ~5000 

# TCP data from rtkrcv (RTKLIB)
class RTKdata:
      def __init__ (self ):
         self.PacketCnt = 0
         self.ValidConn=False
         self.GPSTdate  = ''
         self.GPSTtime  = ''
         self.lat       =                   # y, format llh
         self.long      =                   # x  
         self.height = ''  
         self.Q      = ''                   # 1 - fix, 2 - float (ok  =1;2)
         self.ns     = ''                   # valid satellites  ( Ok >=12
         self.sdn    = ''
         self.sde    = ''
         self.sdu    = ''
         self.sdne   = ''
         self.sdeu   = ''
         self.sdun   = ''
         self.age    = ''      
         self.ratio  = ''                    #  RTKLIB def thresh=3.00
         
      def return_class_variables(self):
          return(self.__dict__)   

# Global parameters
class Parametrai: 
      def __init__ (self ): 
        self.start    = False
        self.pause    = False
        self.stop     = False
        self.auto     = False
        self.shutdown = False
        self.CuttMot  = 0x00
        self.manual   = 0
        self.Jangle   = 0
        self.Jforce   = 0

        self.compasss_corection  = 5          # Magnetic declination ( set by location )  # 8.0              
        self.SpeedV              = 0          # linear velocity  m/s
        self.SpeedW              = 0          # angular velocity rad/s
        self.SpeedV_meas         = 0
        
        self.TargetSpeed         = 0.5        # m/s  (1 m/s - 3.6 km/h)
        self.k_td                = 1.4        # theta_d  lateral Error gain   
        self.k_te                = 0.65       # theta_e  yaw     Error gain   
        self.Kstear              = 0.03       # steering overshoot compensation 

        self.Waypoints           = []
        self.TargetIndex         = 0          # running index (to gui)
        self.last_TargetIndex    = 0          # last acquired index 
        self.Max_targetIndex     = 0          # last target index from Waypoints
        self.DistToTarget        = 0
        self.DistToTargetmin     = 0
        self.Safe_zone           = True       # safe zone between Waypoints in auto mode 
        self.GNSS_valid          = False      # GNSS signall validation
        self.Robot_status        = True       # combined error
        self.PathLateralError    = 0            
        self.WaypointSelected    = 'Waypoints/TestWaypoints.geojson'  # selected from GUI
        self.NewSelectedWaypoint = False
       
      def return_class_variables(self):
          return(self.__dict__)

# For logging data          
class DataRecord:
    def __init__ (self ): 
        self.log1=[{
                    "lat"        : 0,
                    "long"       : 0,
                    "Q"          : 0,
                    "yaw"        : 0, 
                    "v"          : 0,
                    "w"          : 0,
                    "Tidx"       : 0,
                    "Theta_e"    : 0,
                    "Theta_d"    : 0,
                    "DistToTarg" : 0,
                    "latErr"     : 0,

                  }] 
        
    def AddRecord(self, lat1=0, long1=0, Q=0, yaw=0, v=0, w=0, Tidx=0, DistToTarg=0, Theta_e=0, Theta_d=0, lat_err=0):
        self.log1.append({ 
                           "lat"        : lat1,
                           "long"       : long1,
                           "Q"          : Q,
                           "yaw"        : yaw,
                           "v"          : v,
                           "w"          : w,
                           "Tidx"       : Tidx,
                           "DistToTarg" : DistToTarg,
                           "Theta_e"    : Theta_e,
                           "Theta_d"    : Theta_d, 
                           "latErr"     : lat_err,  
                          })

    def return_class_variables(self):
         return(self.__dict__)

def WriteLogToFile(to_csv):

    keys= ['lat', 'long', 'Q', 'yaw','v', 'w','Tidx','DistToTarg','Theta_e','Theta_d']

    with open('RLog-'+timestr+'.csv', 'a', newline='') as output_file:
        dict_writer = csv.DictWriter(output_file, keys)
        dict_writer.writeheader()
        dict_writer.writerows(to_csv)
    
def shutdown():
    if (param.shutdown):
      os.system('sudo shutdown now')

#-------- SSH Keybord for test ----
def press(key):
    #print(f"'{key}' pressed")
    if (key=='up'):     roverpos.lat+=0.000001
    if (key=='down'):   roverpos.lat-=0.000001

    if (key=='left'):   roverpos.long-=0.000001
    if (key=='right'):  roverpos.long+=0.000001 


def release(key):
    #print(f"'{key}' released")    
    b=1   
    
#---------------------------------
cnt1=0
if __name__ == '__main__' :
    now = datetime.datetime.now()
    print("--------------")
    print("RTK robot v1.4")
    print(now.strftime("%Y-%m-%d %H:%M:%S"))

    rx =       RXData()           #  MCU Serial data
    tx =       TXData()
    roverpos = RTKdata()          #  GNSS receiver data
    tmp =      Temp()
    param =    Parametrai()
    DataRec =  DataRecord()

    lock =threading.Lock()
    Serialthr1 =   rserial(shared=rx, shared2=tx, shared3=param )
    TcpSocket = TCPconnect(shared=roverpos, shared2=DataRec, shared3=param)
    web1 =        robotweb(shared=roverpos, shared2=rx, shared3=param, shared4=DataRec)
    navi =       Robotmove(shared=roverpos, shared2=rx, shared3=param, shared4=DataRec, shared5=tx)

    t1 = threading.Thread(target=Serialthr1.RXdata, daemon=True, args=(lock,))
    t2 = threading.Thread(target=TcpSocket.run, daemon=True )
    t3 = threading.Thread(target=web1.run, daemon=True)
    t4 = threading.Thread(target=navi.run, daemon=True)

    t1.start()   # Robot MCU serial com    ( RX interval 100 mS )
    t2.start()   # GNSS reciver TCP client ( TCP socket  data interval 5 Hz )
    t3.start()   # HTTP GUI server
    
    time.sleep(0.5)           # temp, to fix 
    t4.start()                # Control & Automatic navigation
   
    while(1):
        
        #--- For test, ssh terminal keybord input, Blocking code when used !!!
        # listen_keyboard(
        #                  on_press=press,
        #                  #on_release=release,
        #                  delay_second_char=0.05,
        #                  delay_other_chars=0.05,
        #                )
        
        if (rx.Recived==1):
            #print(rx.__dict__)
            rx.Recived=0
            while ( rx.Recived!=0 ):                    # If thread is still writing, wait
                if(lock.locked()==False): rx.Recived=0
        
        time.sleep(0.01)
        shutdown();
        