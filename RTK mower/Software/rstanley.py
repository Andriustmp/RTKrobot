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
import utm
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

point_reached=0.10             # accept waypoint from this distance  0.10m

Kp = 0.25                      # PI speed controll  P 0.25
Ip = 0.01                      # PI speed controll  I 

SafeZone=0.6                   # (m)

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super().__init__()
        self.x               = x
        self.y               = y
        self.yaw             = yaw
        self.deltapast       = 0
        self.v               = v
        self.target_idx      = 1         # first robot target point,  (start location index =0 )

        self.k_td            = 1.4       # theta_d ( atstumo kor.) lateral Error gain   
        self.k_te            = 0.65      # theta_e ( kampo kor. )  yaw     Error gain  
        self.Kstear          = 0.03      # steering overshoot compensation

        self.DistSegment     = 0         # current path length
        self.Dist_safe       = True      # safety coridor on working path
        self.DistToTarg      = 0         # distance (m) to next target
        self.DistToPrevTarg  = 0         # distance (m) to previuosly target
        self.DistToTargMin   = 99999
        self.TurnActive      = False
        self.TargetSpeed     = 0.5       # 0.5 [m/s]  ( update from GUI )
        self.TargetSpeed_set = 0
        self.lateralError    = 0
        self.theta_e         = 0  
        self.theta_d         = 0
        self.integral        = 0

    def update(self, long1, lat1 , yaw, speed, target_speed, k_td, k_te, Kstear):   
        self.x      = long1 
        self.y      = lat1
        self.yaw    = normalize_angle(np.radians(yaw))
        self. v     = speed 
        self.TargetSpeed_set = target_speed
        self.k_td   = k_td
        self.k_te   = k_te
        self.Kstear = Kstear

#--------- PI for speed controll ------------
def pid_control(target, current):
    speed = current
    dif   = target-current
    speed = current+dif*Kp

    if (current<target) : speed += Ip
    if (current>target) : speed -= Ip
    if (target==0)      : speed = 0 

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
     
    Ke = state.k_te
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy, last_target_idx)

    if (current_target_idx >=last_target_idx):
        return  0, current_target_idx
   
    yaw_dif     = abs((cyaw[current_target_idx]+np.pi)-(state.yaw+np.pi))    
    yaw_dif_rev = np.pi-yaw_dif

    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)     #  (output:  -pi : +pi) 
    
    if (abs(yaw_dif_rev)<0.5):                                          # if robot facing oposite to course   (~ 28 deg window)  
         theta_e = theta_e-yaw_dif_rev                                   
        
    if (state.TargetSpeed <= 0.2):                                      # only in slow speds, more aggressive turn
        Ke = 1.4                                                        # ok for slow speed  
    else:
        Ke = state.k_te                                                 # back to normal gain

    if ((state.TurnActive==True)and( yaw_dif>1.35)):                    # near target and >=77 deg 

        state.TargetSpeed = 0
        if(theta_e>0): delta = 1
        if(theta_e<0): delta = -1
        state.deltapast = 0
        
    else:  
            state.TurnActive=False  
            theta_d = np.arctan2(state.k_td * error_front_axle, 0.1+state.v)    
            # Steering control
            delta = theta_e*Ke + theta_d 
            delta = delta+ state.Kstear*(delta-state.deltapast)
            state.theta_e = theta_e
            state.theta_d = theta_d
            state.deltapast = delta
        
            #print("Delta:{:.3f} ".format(delta), "theta_e:{:.3f} ".format(theta_e)," theta_d:{:.3f}" .format(theta_d) )  
    return delta, current_target_idx

#-------------------------------------
def calc_target_index(state, cx, cy, last_target_idx):
   
    old_idx = state.target_idx

    state.DistToTarg     = distance(state, state.y, state.x, cy[state.target_idx], cx[state.target_idx] ) 
    state.DistToPrevTarg = distance(state, state.y, state.x, cy[state.target_idx-1], cx[state.target_idx-1] )
    state.DistSegment    = distance(state, cy[state.target_idx], cx[state.target_idx], cy[state.target_idx-1], cx[state.target_idx-1] )
    
    TargetColect(state)  

    if (state.target_idx >=last_target_idx) :
        return  state.target_idx, 0

    if(old_idx!=state.target_idx):
        state.DistToTarg     = distance(state, state.y, state.x, cy[state.target_idx], cx[state.target_idx] ) 
        state.DistToPrevTarg = distance(state, state.y, state.x, cy[state.target_idx-1], cx[state.target_idx-1] )
        state.DistSegment    = distance(state, cy[state.target_idx], cx[state.target_idx], cy[state.target_idx-1], cx[state.target_idx-1] ) 
        old_idx = state.target_idx

    
    state.lateralError = distanceLineInfinite(state.x, state.y, cx[state.target_idx-1], cy[state.target_idx-1], cx[state.target_idx], cy[state.target_idx]); 
    error_front_axle = state.lateralError*-1
    #print("error_front_axle:{:.2f} ".format(lateralError))
     
    DistanceSafety(state)
     
    return state.target_idx, error_front_axle

# ----------------------
def TargetColect(state):

       state.TargetSpeed = 0.2 

       if(state.DistToTarg>2)and(state.DistToPrevTarg>0.8):    # back to GUI set speed
           state.TargetSpeed = state.TargetSpeed_set        

       if(state.DistToTarg<1.5):  state.TargetSpeed = 0.4  
          
       if(state.DistToTarg<0.7):  state.TargetSpeed = 0.2  
          
       if(state.DistToTarg<=point_reached):                    # <10 cm
          print("target acquired  (target_idx, DistToTarg): ", state.target_idx, state.DistToTarg) 
          state.target_idx += 1
          state.TurnActive = True
          state.DistToTargMin = 99999
          
       else:
            if ((state.DistToTarg<0.6)and(state.TurnActive==False)):                         # If going to miss target  
                if(state.DistToTarg>state.DistToTargMin):
                   print("target missed (target_idx, DistToTarg): ",state.target_idx, state.DistToTarg) 
                   state.target_idx += 1
                   state.TurnActive = True
                   state.DistToTargMin = 99999
                else:   
                      if (state.DistToTargMin> state.DistToTarg ): 
                          state.DistToTargMin= state.DistToTarg 
                    
# compute distance to (infinite) line (https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points)
def distanceLineInfinite(px, py, x1, y1, x2, y2):
    
    p1 = []
    p1 = utm.from_latlon(py,px)     # UTM 34U
    px = p1[0]
    py = p1[1]

    x_1 = []
    x_1 = utm.from_latlon(y1,x1)
    x1  = x_1[0]
    y1  = x_1[1]

    x_2 = []
    x_2 = utm.from_latlon(y2,x2)
    x2  = x_2[0]
    y2  = x_2[1]

    len1 = math.sqrt((y2-y1)**2+(x2-x1)**2)

    if (abs(len1) < 0.00000000001): 
       distToLine=0
    else: distToLine = ((y2-y1)*px-(x2-x1)*py+(x2*y1-y2*x1)) / len1

    return distToLine    # (m)

# ----------------------------    
def DistanceSafety(state):    
    safe = True 

    if (abs(state.lateralError) > (SafeZone) ): 
        safe = False  
        print("Safe zone failed (target_idx, lateral_Error): ", state.target_idx, state.lateralError)

    zone_forward = state.DistToPrevTarg-state.DistSegment    
    zone_reverse = state.DistToTarg-state.DistSegment

    if (zone_forward >SafeZone):
        safe = False  
        print("Safe zone failed (target_idx, zone_forward): ", state.target_idx, zone_forward) 

    if (zone_reverse >SafeZone):
        safe = False  
        print("Safe zone failed (target_idx, zone_reverse): ", state.target_idx, zone_reverse)
    
    state.Dist_safe = safe
    return safe   


def normalize_angle(angle):  
    # Normalize an angle to [-pi, pi]
    mod_angle = (angle + np.pi) % (2 * np.pi) - np.pi
    return  mod_angle 

#------------- Calc yaw for waypoints pairs ----     
def yaw_calc(state, cx, cy, yaw):         
    yaw_p = [0]  
    for i in range(0,len(cx)-1):

          long1 = cx[i]
          lat1  = cy[i]
          long2 = cx[i+1] 
          lat2  = cy[i+1]

          dLon = (long2 - long1)
          x = math.cos(math.radians(lat2)) * math.sin(math.radians(dLon))
          y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dLon))
          brng = np.arctan2(x,y)
          yaw_p.append(round(brng,3))
           
    return  yaw_p

def calcmotion(state, cx, cy, cyaw):
       
     last_idx = len(cx)    
     
     if  state.target_idx < last_idx:                 
        di, state.target_idx = stanley_control(state, cx, cy, cyaw, last_idx)     # return delta, current_target_idx
        ai = pid_control(state.TargetSpeed, state.v)
        return ai, di, state.lateralError, state.Dist_safe                        # Return Linear (V) and angular(W)  to motors control

     else: 
        return 0, 0, 0, True    