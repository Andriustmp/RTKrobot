
"""
 RTK robot mower V 1.1
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

point_reached=0.10              # accept waypoint from this distance  0.02-0.1m
k  = 0.0013                     # theta_d (atstumo kor.) control gain  0.0013
Kp = 0.25                       # PI speed controll  P 0.1
Ip = 0.01                       # PI speed controll  I 
L = 1.0                         # [m] Wheel base of vehicle  ( atstumas tarp asiu )

max_steer = np.radians(150.0)   # [rad] max steering angle   

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super().__init__()
        self.x= x
        self.y= y
        self.yaw= yaw
        self.deltapast= 0
        self.v= v

        self.target_idx= 1        # zero index - first robot point from start location.
        self.DistToTarg= 0        # distance (m) to next target
        self.DistToPrevTarg= 0    # distance (m) to previuosly target
        self.DistToTargMin=99999
        self.TargetSpeed=0.4      # 0.4 [m/s]
        self.theta_e= 0
        self.theta_d= 0
        self.integral=0

    def update(self, x, y , yaw, v):   
       
        self.x= x 
        self.y= y
        self.yaw= normalize_angle(np.radians(yaw))
        self.v= v    


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

#  --------Stanley steering control-----------
def stanley_control(state, cx, cy, cyaw, last_target_idx):
     
    Ke=0.9
    Kstear=0.1

    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)
    # test
    #state.yaw=cyaw[current_target_idx]

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    print("Current_Target_Idx: ", current_target_idx)
    print("CurTarg_cyaw:",cyaw[current_target_idx])
    print("yaw (rad):", state.yaw)

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)    #  (output:  -pi : +pi) roboto pozicija nesutampa su target tasku ! (  nebutina normalize ? )

    if (state.TargetSpeed == 0.2):                                     # only in slow speds, more aggressive turn
        Ke=1
    else:
        Ke=0.8

    print("theta_e (kampo kor.):",theta_e )

    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, 0.1+state.v)            # state.v  # (arctan2- vector to an angle ) ( arctan output:  -pi/2 : +pi/2 )
    #theta_d=0
    print("theta_d (atstumo kor.):",theta_d )

    # Steering control
    delta = theta_e*Ke + theta_d 
    delta = delta+ Kstear*(delta-state.deltapast)
    #print("delta:",delta)

    state.theta_e=theta_e
    state.theta_d=theta_d

    state.deltapast=delta
    
    return delta, current_target_idx

#-------------------------------------
def calc_target_index(state, cx, cy):
 
    # Calc front axle position
    fx = state.x 
    fy = state.y 
   
    # Search nearest point index
    dx = [fx - icx for icx in cx]            # change from front axis to each waypoint
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)                    
  
    #dist_to_target = np.sqrt((state.x -cx[state.target_idx])**2+(state.y -cy[state.target_idx])**2)

    state.DistToTarg     = distance(state, state.y, state.x, cy[state.target_idx], cx[state.target_idx] ) 
    state.DistToPrevTarg = distance(state, state.y, state.x, cy[state.target_idx-1], cx[state.target_idx-1] ) 

    #print("Distance to Prev target : ",state.DistToPrevTarg)
    
    TargetColect(state)     

    print("dist_to_target:",state.DistToTarg)
    print("dist_to_target_MIN:",state.DistToTargMin)

    lateralError = distanceLineInfinite(state.x, state.y, cx[state.target_idx-1], cy[state.target_idx-1], cx[state.target_idx], cy[state.target_idx]);
    lateralError = lateralError *75000000    # DD -> mm: 0.000001 ->75 mm

    error_front_axle=lateralError*-1
    
    return state.target_idx, error_front_axle


def TargetColect(state):

       if(state.DistToTarg>2)and(state.DistToPrevTarg>0.8):   state.TargetSpeed = 0.5  

       if(state.DistToTarg<1.5):  state.TargetSpeed = 0.4

       if(state.DistToTarg<0.7):  state.TargetSpeed = 0.2
      

       if(state.DistToTarg<=point_reached):               # <10 cm
          state.target_idx+=1
          state.DistToTargMin=99999
          
       else:
            if (state.DistToTarg<0.4):                    # If giong to miss target
                if(state.DistToTarg>state.DistToTargMin):
                   state.target_idx+=1
                   state.DistToTargMin=99999
                   
                if (state.DistToTargMin> state.DistToTarg ): 
                      state.DistToTargMin= state.DistToTarg 
                    
#------------------------------------------------------

# compute distance to (infinite) line (https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points)

def distanceLineInfinite(px, py, x1, y1, x2, y2):

    len1 = math.sqrt((y2-y1)**2+(x2-x1)**2)

    if (abs(len1) < 0.00000000001): 
       distToLine=0

    else: distToLine = ((y2-y1)*px-(x2-x1)*py+(x2*y1-y2*x1)) / len1

    return distToLine

# -------- distance from point to line (v, w)    
def distanceLine(px, py, x1, y1, x2,  y2):          
    x3 = px
    y3 = py
    lx=x2-x1
    ly=y2-y1
    temp=(lx*lx)+(ly*ly)
    u=((x3 - x1) * lx + (y3 - y1) * ly) / (temp)

    if(u>1): u=1
    if(u<0): u=0
    
    x = x1 + u * lx
    y = y1 + u * ly
    dx = x - x3
    dy = y - y3
    dist = math.sqrt(dx*dx + dy*dy);
    return dist           

def normalize_angle(angle):  
    #Normalize an angle to [-pi, pi].
    mod_angle = (angle + np.pi) % (2 * np.pi) - np.pi
    return  mod_angle 

#------------- calc yaw for waypoints pairs --------     
def yaw_calc(state, cx, cy, yaw):         
    yaw_p=[0]    
    for i in range(1,len(cx)):

          long1=cx[i-1]
          long2=cx[i]
          lat1 =cy[i-1]
          lat2 =cy[i]
 
          dLon = (long2 - long1)
          x = math.cos(math.radians(lat2)) * math.sin(math.radians(dLon))
          y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dLon))
          brng = np.arctan2(x,y)
          #brng = np.degrees(brng)        # deg -180 + 180
          yaw_p.append(round(brng,3))
       
    print("First point cy0: ",cy[0])      # to debug
    print("First point cx0: ",cx[0])
    print("Sec   point cy1: ",cy[1])
    print("Sec   point cx1: ",cx[1])    

    return  yaw_p

def calcmotion(state, cx, cy, cyaw):
       
     last_idx = len(cx) - 1

     if last_idx > state.target_idx:                 # last point ignored, to fix   !!!!!
        
        ai = pid_control(state.TargetSpeed, state.v)
        di, state.target_idx = stanley_control(state, cx, cy, cyaw, state.target_idx)     # return delta, current_target_idx
 
        return ai, di                                # Return Linear (V) and angular(W)  to motor control

     else: 
        return 0, 0    