# RTK lawn robot mower
![RTKmower_foto](./RTKmower_foto.jpg)

This autonomous lawn robot project is based on RTK navigation using its own RTK base station without the use of a ground wire. Uses only a closed local WiFi network for communication. Uses widely available components, so the configuration can be customized to your needs. Waypoints for work zones are semi-manually placed in QGIS software and exported to the robot in GeoJSON format. This is not a turnkey product, you will need to customize it to your needs. 


**Project objectives:**

- rtklib-based autonomous lawn robot
- Using NEO-M8T GNSS receivers
- Python main programming language 
- WEB-based user interface  ( Dash framework for GUI )
- Robust design for larger areas ( minimum 4 hours of work per charge )


**Hardware:**

How to compile arduino code:

- Install required libraries:
  - https://github.com/rogerclarkmelbourne/Arduino_STM32
  - https://github.com/jrowberg/i2cdevlib
  - https://github.com/DFRobot/DFRobot_LSM303
- Copy customized library files to LSM303 lib, from „Customized LSM303 lib“ folder
- The STlink adapter is used to load the program
   
**Software:**\
(RTKbase)

- With UBLOX u-center software load  NEO-M8T configuration from: „RTKrobot\NEO-M8T module config (u-center)\NEO-M8T  config Gps Galileo BeiDou 5Hz“
  ( change update rate to  1Hz )
- Git clone https://github.com/rtklibexplorer/RTKLIB/releases/tag/b34h
- Build the code, only the str2str app will be used
- Create and run the systemd service: „RTKrobot\Base station\rtkbase.service“

(RTKmower)

- With UBLOX u-center software load  NEO-M8T configuration from: „RTKrobot\NEO-M8T module config (u-center)\NEO-M8T  config Gps Galileo BeiDou 5Hz“
- Create Wifi client to RTKbase station 
- Clone, build rtklibexplorer software package (rtkrcv app)
- Copy and edit „rtkrcv“ configuration file from: „RTKrobot\RTK mower\rtklib rtkrcv config“ (ant2-pos1; ant2-pos2 )
- Copy and run the systemd service: „RTKrobot\RTK mower\rtkrover.service“
- Create yard perimeter and waypoint drawings, export in WGS84 GeoJSON format
- Upload your yard perimeter drawing to directory: „\assets“
- Upload waiponts to directory: „\Waypoints“
- Run main python file „RTKrobot\RTK mower\Software\robotmain.py“
- Graphical user interface is accessible via robotIP:8050

**Highlights:**

- In the QGIS software, create a grid of 25x25 cm (depending on the width of the robot). Based on it, you can easily draw waypoints in the next layer that the robot will use.
- From the launch position, the robot moves in a straight line towards the first waypoint.
- Since the robot uses a tilt compensated compass, it is very important to properly calibrate the LSM303 magnetometer and accelerometer.


**Todo list:**

- [x]  Add obstacle avoidance function ( only stop )
- [ ]  Add battery monitoring function to GUI
- [ ]  Fix the event logging part 
- [ ]  combine wheel encoder data with gnss position data for more accurate navigation
- [x]  Adapt the control algorithm for sharp turns ( the stanley part works perfectly in straight lines, sharp angles are smoothed)

**References:**

https://github.com/AtsushiSakai/PythonRobotics

https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf

https://rtklibexplorer.wordpress.com/

 
  
