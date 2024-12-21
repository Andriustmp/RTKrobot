# RTK lawn robot mower
![RTKmower_foto](./RTKmower_foto.jpg)

This autonomous lawn robot project is based on RTK navigation using its own RTK base station without the use of a ground wire. Uses only a closed local WiFi network for communication. Uses widely available components, so the configuration can be customized to your needs. Waypoints for work zones are semi-manually placed in QGIS software and exported to the robot in GeoJSON format. This is not a turnkey product, you will need to customize it to your needs. 


Project objectives:

- rtklib- based autonomous lawn robot
- Using NEO-M8T GNSS receivers 
- Robust design for larger areas ( minimum 4 hours of work per charge )
- Hardware WDT if main program is down 
- Python main programming language 
- Hardware part based on arduino  ( STM32 Blue Pill )
- WEB-based user interface  ( Dash framework for GUI )
