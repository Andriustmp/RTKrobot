
"""
 RTK robot mower V 1.1
 Robot manual/automatic movment controll
 2024 05 23
 author: A. Besusparis
"""

import threading
import json
import geojson
import time
import datetime
import math
import numpy as np

import  rstanley


class Robotmove(threading.Thread): 

   def __init__(self,shared, shared2, shared3, shared4, shared5): 
          self.shared1 = shared   # RTK data
          self.shared2 = shared2  # MCU data
          self.shared3 = shared3  # parametrai
          self.shared4 = shared4  # Data_log
          self.shared5 = shared5  # TX to MCU

          self.pozicija=[54.0, 23.0]
          #----  
          self.Oldtime=0
          self.Oldtime2=0
          
          self.tick=False
          self.Validation=False
          self.direction='F'      # manual mode F/R
          
          self.manual_trig=False
          self.oldcntL=0
          self.oldcntR=0
          self.oldpacketcnt=0
          self.oldpacketcnt2=0

          self.r=0.125            #  wheel radius (m)
          self.L=0.45             #  distance between wheels (m); in Auto driving mode: influences the direction of the wheel ( 0.45 -0.85 )

          # Initial robot state
          self.state = rstanley.State(x=0.0, y=0.0, yaw=np.radians(0.0), v=0.0)
          self.cx=[]
          self.cy=[]
          self.cyaw=[]


   @staticmethod
   def check_float(value):
       if value is None: return 0
       try:
           float(value)
           return  float(value)
       except ValueError:
           return 0     
      
   def get_sec(self,time_str):
       # separate  hh, mm, ss
       hh, mm, ss = time_str.split(":")
       return int(hh) * 3600 + int(mm) * 60 + int(ss)
   
   def GNSSvalidation(self):             # ---------- to optimize !!!
       Newtime=0
       Timedif=0
       RecTimeS=0
       #--------- tick 1s----------   
       Newtime=time.monotonic()
       if (self.Oldtime==0 ): self.Oldtime=Newtime   # only 1-st run
       Timedif=self.Oldtime-Newtime

       if (abs((Timedif))>=1.0):   
           self.tick=True
           self.Oldtime=time.monotonic()  
       #------------------------
       if(self.tick==True):
           #print("1s...")
           # GNSS time must change every second 
           if len(self.shared1.GPSTtime)>0: 
               RecTimeS=self.get_sec(self.shared1.GPSTtime[:-4])
               # GNSS reciver time is updated
               if(self.Oldtime2!=RecTimeS): 
                   #print(RecTimeS)                                                           #  to fix
                   # RTKLIB solution Q: fix=1; float-2,  valid sat>=8, solution ratio >=1.5
                   if(self.shared1.Q==1)and(self.shared1.ns>=8)and(self.shared1.ratio>=1.5):  # add: sdn, sde - stabilitys for 3s (<0.08) 
                       # print("GNSS data valid:")
                        self.Validation=True
                   else:
                        print("GNSS data not valid:")
                        self.Validation=False    
               else:
                   self.Validation=False
                   print("FAULT: GNSS Time not changing")                                     # GNSS receiver or RTKlib TCP stream fault
               self.Oldtime2=RecTimeS   
    
           else:
               self.Validation=False
               print("FAULT: GNSS Time no data")

           self.tick=False 


   def SpeedMeasure(self):
       impmm_scale=1.0738

       if (self.shared2.PacketCnt>self.oldpacketcnt):                               # @200 ms RX update

           diffL=abs(self.shared2.LWheelCnt-self.oldcntL)
           self.oldcntL=self.shared2.LWheelCnt
              
           diffR=abs(self.shared2.RWheelCnt-self.oldcntR)
           self.oldcntR=self.shared2.RWheelCnt

           self.shared2.SpeedV=round(((((diffL+diffR)/2)*impmm_scale)/1000)*5, 2)   #  Linear speed from wheel encoders (m/s)

           #print("SpeedV:",self.shared2.SpeedV)
           self.oldpacketcnt=self.shared2.PacketCnt 


           #----- Log data ------
           self.shared4.AddRecord(
                                  self.shared1.lat,
                                  self.shared1.long,
                                  self.shared1.Q,
                                  rstanley.normalize_angle(np.radians(self.shared2.Direction)),
                                  self.shared3.SpeedV,
                                  self.shared3.SpeedW,
                                  self.shared3.TargetIndex,
                                  self.state.DistToTarg,
                                  self.state.theta_e,
                                  self.state.theta_d,
                                 )

           return 1
 

 # Wheel length 0.8m -  745 pulses/s= 2pi rad/s
 # 118.57 imp/s = 1 rad/s
 # imp - Wheel encoder pulses /100 ms

   def Motors(self, v, w, motdir="F", mode="auto"):      # motor dir -"F" or "R"
       
        Lmotdir ='F'
        Rmotdir ='F'      
        Wl=(v+w*self.L/2)/self.r  
        Wr=(v-w*self.L/2)/self.r                         # rad/s
        #print("Wl-------:",Wr)
        #print("Wr-------:",Wl)     
          
        if (mode=="manual"):

            if(Wl<0): Wl=0
            if(Wr<0): Wr=0  

            impL=int((Wl*118.57)/10)                     #  118.57 imp/s = 1 rad/s             
            impR=int((Wr*118.57)/10)                     #  pulses/100ms
            
            #print("impL:",impL)   
            if (impL >255) : impL=255
            if (impR >255) : impR=255
            if (impL <0) : impL=0
            if (impR <0) : impR=0

            if (self.shared5.Transmit==0):
                         self.shared5.LmotorFR=motdir  
                         self.shared5.Lspeed = impL
                         self.shared5.RmotorFR=motdir
                         self.shared5.Rspeed = impR
                         self.shared5.RunStop= self.shared3.CuttMot
                         self.shared5.Transmit=1
                         
        else:                                           # Auto mode 
       
           if (Wl>=0): LmotDir='F'
           else:       LmotDir='R'
           impL=abs(int((Wl*118.57)/10)) 

           if (Wr>=0): RmotDir='F'
           else:       RmotDir='R'
           impR=abs(int((Wr*118.57)/10))

           if (impL >255) : impL=255
           if (impR >255) : impR=255
           if (impL <0) : impL=0
           if (impR <0) : impR=0       

           if (self.shared5.Transmit==0):
                         self.shared5.LmotorFR=LmotDir  
                         self.shared5.Lspeed = impL
                         self.shared5.RmotorFR=RmotDir
                         self.shared5.Rspeed = impR
                         self.shared5.RunStop= self.shared3.CuttMot
                         self.shared5.Transmit=1 

        #time.sleep(0.01)

# ---------------------- Wayponts --------------------

   def loadGeojsonWaypoints(self):

        geojson.geometry.DEFAULT_PRECISION = 7                     # ~1 cm accuracy
        with open(self.shared3.WaypointSelected) as file:
           gj = geojson.load(file)
        
        self.cx=[]
        self.cy=[]
        self.cyaw=[]
        pointdata = gj['features'][0]['geometry']['coordinates'][0]
        self.shared3.Waypoints=pointdata
        print("Loaded  Waypoints: ", self.shared3.WaypointSelected)
        #print("wayponts (x,y):",self.shared3.Waypoints)     
 
   def WaypointPrepare(self):

        for item in self.shared3.Waypoints:
            self.cx.append(item[0])
            self.cy.append(item[1])
        
        self.cx.insert(0, float(self.shared1.long))                # first point robot start location
        self.cy.insert(0, float(self.shared1.lat)) 

        self.cyaw=rstanley.yaw_calc(self.state, self.cx, self.cy, self.shared2.Direction) 

        print("cx:",self.cx)
        print("cy:",self.cy)
        print("cyaw:",self.cyaw)

  #----------------  Drive modes ------------- 

   def AutomaticDriveMode(self):
      # self.Validation=True
      # self.shared2.Direction
      # self.shared3.SpeedV
      # self.shared3.SpeedW

      if ( self.shared3.start==True):

          rstanley.State.update(self.state, self.shared1.long, self.shared1.lat, self.shared2.Direction, self.shared2.SpeedV ) 

          self.shared3.SpeedV, self.shared3.SpeedW = rstanley.calcmotion(self.state, self.cx, self.cy, self.cyaw )

          self.shared3.TargetIndex=self.state.target_idx  # to web

          print("speed V: ",self.shared3.SpeedV )
          print("speed W: ",self.shared3.SpeedW )
          
          if ( self.shared3.pause==False):
              self.Motors(self.shared3.SpeedV, self.shared3.SpeedW)
          else:
              self.Motors(0,0) 

   def ManualDriveMode(self):
       #print("kampas",self.shared3.Jangle)
       #print("jega",self.shared3.Jforce)
       speedmanual=0
       radmanual=0
       force=self.check_float(self.shared3.Jforce)

       if(self.shared3.manual==True):
           self.manual_trig=True
           if (force!=0):
              if (self.shared5.Transmit==0):

                  if(self.shared3.Jangle<=180):                              # go Forward
                      self.direction='F'
                      radmanual=math.radians(90-self.shared3.Jangle)         # joistick shift 90 deg 
                      speedmanual=force
                  else: 
                      self.direction='R'                                     # go Reverse
                      radmanual=math.radians(270-self.shared3.Jangle)*-1
                      speedmanual=force

                  if(speedmanual>0.5): speedmanual=0.5              
                  self.Motors(speedmanual,radmanual, self.direction, "manual") 

                  #print("speed m/s:",speedmanual)
                  #print("angle rad :",radmanual) 
           else:
               self.Motors(0,0, "S")                                         # joistick inactive
                 

       else:
            if (self.shared3.manual==False)and(self.manual_trig==True):      # Stop motors after manual off
                if (self.shared5.Transmit==0):

                    self.shared5.LmotorFR="F"
                    self.shared5.RmotorFR="F"
                    self.shared5.Lspeed = 0
                    self.shared5.Rspeed = 0

                    self.shared5.Transmit=1
                    self.manual_trig=False

       time.sleep(0.02)
       return 1      

#----------------------------------------------        
   def run (self):  

       self.loadGeojsonWaypoints() 
       self.WaypointPrepare()

       while(1):


          if (self.shared3.NewSelectedWaypoint==True):
              self.loadGeojsonWaypoints() 
              self.WaypointPrepare()
              self.shared3.NewSelectedWaypoint=False
              
          
          self.SpeedMeasure()
          #self.GNSSvalidation()
          self.ManualDriveMode()

          if(self.shared3.manual==False)and(self.shared3.start==True): 
             if (self.shared2.PacketCnt>self.oldpacketcnt2):           # only after new data from MCU  (~200 ms)
                   self.AutomaticDriveMode()
                   self.oldpacketcnt2=self.shared2.PacketCnt
          else :  

             if ( self.shared3.manual==False): self.Motors(0, 0,"F")   # Start not active, or Stop button pressed
            

         # self.Motors(0.2, 0.0,"F")       # 1 m/s -> 3.6 km/h
         #  self.shared2.AddRecord(RTKdata[2],RTKdata[3],RTKdata[5])

          time.sleep(0.02) #20 ms refresh rate       