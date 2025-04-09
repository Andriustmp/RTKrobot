"""
 RTK robot mower V 1.4
 Stanley control Path tracking ( with WGS84 Coordinate System)
 author: A.Besusparis

 Ref:
    - [ref code] https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/stanley_controller
    - [Stanley: The robot that won the DARPA grand challenge] (http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
    - [Autonomous Automobile Path Tracking] (https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)
"""

import math 
import numpy as np
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

point_reached=0.10             # accept waypoint from this distance  10 cm...
k  = 0.0013                    # theta_d (atstumo kor.) control gain  0.0013
Kp = 0.25                      # PI speed controll  P 0.1
Ip = 0.01                      # PI speed controll  I 

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super().__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.deltapast = 0
        self.v = v
        self.target_idx = 42       # first robot target point,  (start location index =0 )
        self.DistToTarg = 0        # distance (m) to next target
        self.DistToPrevTarg = 0    # distance (m) to previuosly target
        self.DistToTargMin = 99999
        self.TargetSpeed = 0.4      # 0.4 [m/s]
        self.TargetSpeed_set = 0
        self.theta_e = 0
        self.theta_d = 0
        self.integral =0

    def update(self, long1, lat1 , yaw, speed, target_speed):   
      
        self.x= long1 
        self.y= lat1
        self.yaw= normalize_angle(np.radians(yaw))
        self.v= speed 
        self.TargetSpeed_set=target_speed


#--------- PI for speed controll ------------
def pid_control(target, current):
    speed=current
    dif=(target-current) 
    speed=current+dif*Kp

    if(current<target) : speed+=Ip
    if(current>target) : speed-=Ip

    return speed


def distance(state, lat1, lon1, lat2, lon2):     # lat - y; long - x 
    radius =6378137                              # 6371000 Earth radius (m)

    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) * math.sin(dlat / 2) +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(dlon / 2) * math.sin(dlon / 2))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    d = round(radius * c, 4)

    return d

#  -------------Stanley steering control-----------
def stanley_control(state, cx, cy, cyaw, last_target_idx):
     
    Ke     = 0.9         # yaw koefic
    Kstear = 0.06        # buvo 0.1

    current_target_idx, error_front_axle = calc_target_index(state, cx, cy, last_target_idx)
    
    if (current_target_idx >=last_target_idx):
        return  0, current_target_idx
        

    print("Current_Target_Idx: ", current_target_idx)
    print("CurTarg_cyaw:",cyaw[current_target_idx])
    print("yaw (rad):", state.yaw)

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)    #  (output:  -pi : +pi) 

    if (state.TargetSpeed == 0.2):                                     # only in slow speds, more aggressive turn
        Ke=1.5                                                         # ok for slow speed
    else:
        Ke=0.8  

    print("theta_e (kampo kor.):",theta_e )
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, 0.1+state.v)            # state.v  # (arctan2- vector to an angle ) ( arctan output:  -pi/2 : +pi/2 )
    print("theta_d (atstumo kor.):",theta_d )

    # Steering control
    delta = theta_e*Ke + theta_d 
    delta = delta+ Kstear*(delta-state.deltapast)
    state.theta_e=theta_e
    state.theta_d=theta_d
    state.deltapast=delta
    
    return delta, current_target_idx

#-------------------------------------
def calc_target_index(state, cx, cy, last_target_idx):
   
    state.DistToTarg     = distance(state, state.y, state.x, cy[state.target_idx], cx[state.target_idx] ) 
    state.DistToPrevTarg = distance(state, state.y, state.x, cy[state.target_idx-1], cx[state.target_idx-1] ) 

    TargetColect(state)  

    if (state.target_idx >=last_target_idx) :
        return  state.target_idx, 0

    print("dist_to_target:",state.DistToTarg)
    print("dist_to_target_MIN:",state.DistToTargMin)

    print(state.target_idx)

    lateralError = distanceLineInfinite(state.x, state.y, cx[state.target_idx-1], cy[state.target_idx-1], cx[state.target_idx], cy[state.target_idx]);
    lateralError = lateralError *75000000      # DD -> mm: 0.000001 ->75 mm
    
    error_front_axle=lateralError*-1
    #print("error_front_axle:{:.2f} ".format(lateralError))

    return state.target_idx, error_front_axle

# ----------------------
def TargetColect(state):

       if(state.DistToTarg>2)and(state.DistToPrevTarg>0.8):    # back to GUI set speed
           state.TargetSpeed = state.TargetSpeed_set        

       if(state.DistToTarg<1.5):  state.TargetSpeed = 0.4  
          
       if(state.DistToTarg<0.7):  state.TargetSpeed = 0.2  
          
       if(state.DistToTarg<=point_reached):    # <10 cm
          print("target acquired ") 
          state.target_idx+=1
          state.DistToTargMin=99999
          
       else:
            if (state.DistToTarg<0.6):                        # If going to miss target  
                if(state.DistToTarg>state.DistToTargMin):
                   print("target missed") 
                   state.target_idx+=1
                   state.DistToTargMin=99999
                else:   
                      if (state.DistToTargMin> state.DistToTarg ): 
                          state.DistToTargMin= state.DistToTarg 
                    
#---------------------------------------------------

# compute distance to (infinite) line (https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points)
def distanceLineInfinite(px, py, x1, y1, x2, y2):

    len1 = math.sqrt((y2-y1)**2+(x2-x1)**2)

    if (abs(len1) < 0.00000000001): 
       distToLine=0
    else: distToLine = ((y2-y1)*px-(x2-x1)*py+(x2*y1-y2*x1)) / len1

    return distToLine

# ----------------------------    
def distanceSafety(state):    # TODO

    # lateralError <0.5m
    # Target line crossing 
    # prediction time     
    
    return 0           

def normalize_angle(angle):  
    #Normalize an angle to [-pi, pi].
    mod_angle = (angle + np.pi) % (2 * np.pi) - np.pi
    return  mod_angle 

#------------- Calc yaw for waypoints pairs ----     
def yaw_calc(state, cx, cy, yaw):         
    yaw_p=[0]  
    for i in range(0,len(cx)-1):

          long1=cx[i]
          lat1 =cy[i]
          long2=cx[i+1] 
          lat2 =cy[i+1]

          dLon = (long2 - long1)
          x = math.cos(math.radians(lat2)) * math.sin(math.radians(dLon))
          y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dLon))
          brng = np.arctan2(x,y)
          #brng = np.degrees(brng)  # deg -180 + 180
          yaw_p.append(round(brng,3))
           
    print("First point cy0: ",cy[0])  # to debug
    print("First point cx0: ",cx[0])
    print("Sec   point cy1: ",cy[1])
    print("Sec   point cx1: ",cx[1])    


    return  yaw_p

def calcmotion(state, cx, cy, cyaw):
       
     last_idx = len(cx)    
     #print("idxxxx:",last_idx)

     if  state.target_idx < last_idx:                 
        
        ai = pid_control(state.TargetSpeed, state.v)
        di, state.target_idx = stanley_control(state, cx, cy, cyaw, last_idx)     # return delta, current_target_idx
 
        return ai, di                                                             # Return Linear (V) and angular(W)  to motors control

     else: 
        return 0, 0    