 
"""
 RTK robot mower V 1.1
 WEB user interface  x.x.x.x:8050
 2024 05 23
 author: A. Besusparis
"""

import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

import os
import json
import dash_leaflet as dl
import dash_daq as daq
from   dash import Dash, html, Output, Input
from   dash import Dash, html, dcc, callback, Output, Input
from   dash import Dash, dash_table

import threading 
import random
import plotly.graph_objs as go

files = os.listdir("Waypoints")


class robotweb(threading.Thread):

    def __init__(self,shared, shared2, shared3, shared4): 
          self.tmp1=[shared.return_class_variables()]      # display list of dict
          self.shared2 = shared2
          self.shared3 = shared3
          self.shared4 = shared4
          self.pozicija=[54.0, 23.0]
          
          LogData = self.shared4 
          self.Graph1index=1  
          self.DisplayRecords=500
          self.geojson=[]


          with open(self.shared3.WaypointSelected, 'r') as f:
               self.geojson = json.load(f)
          

    def ListFiles(self):
            file_names = os.listdir("Waypoint")
            return file_names      
                
    def getlogdata(self):
        x=[]
        y=[]
        Q=[]
        yaw=[]
        speedV=[]
        speedW=[]
        tindex=[]
        disttotarg=[]
        thetaE=[]
        thetaD=[]

        logitems=len(self.shared4.log1)
        
        if (logitems>self.DisplayRecords): 
           start=logitems-self.DisplayRecords
        else: start=0   

        for index in range(start, logitems ):
           
            yaw.append(self.shared4.log1[index]['yaw'])
            speedV.append(self.shared4.log1[index]['v'])
            speedW.append(self.shared4.log1[index]['w'])
            tindex.append(self.shared4.log1[index]['Tidx'])
            disttotarg.append(self.shared4.log1[index]['DistToTarg'])
            thetaE.append(self.shared4.log1[index]['Theta_e'])
            thetaD.append(self.shared4.log1[index]['Theta_d'])

        return  x, y, Q, yaw, speedV, speedW, tindex, disttotarg, thetaE, thetaD  

    def run(self):
        
        tab_height = '3vh'
        app = Dash(__name__)
        app.layout = html.Div([ 
             dcc.Tabs([ 
                 dcc.Tab(label='Map', style={'padding': '0','line-height': tab_height}, selected_style={'padding': '0','line-height': tab_height},
                 children=[
                     #--------------- 
                     dl.Map( children=[                                                        # The Map object needs a list of element
                                       dl.Marker(position=self.pozicija, id='position-data'),
                                       dl.GeoJSON(url="/assets/Perimetras2.geojson", id="states"),
                                       #dl.GeoJSON(url=self.Waypoint_selected, id='Wmap', options={"style":{"color":" #df8400","weight":"1"}}),  
                                       dl.GeoJSON(data=self.geojson, id='Wmap', options={"style":{"color":" #df8400","weight":"1"}}),
                                      ], 
                                      center=(55.22954933, 25.72507911), zoom=20, style={'height': '60vh'}   
                            ), 
                     #---------------
                     html.Div([
                         html.Div([   
                                  daq.Gauge(
                                   id='RobototGauge1',
                                   showCurrentValue=True,
                                   size=150,
                                   color="#273b7d",
                                   value=self.shared2.Direction,
                                   label='Robot heading (deg)',
                                   max=360,
                                   min=0,
                                   style={'left': '10px'}     # 'width': '40%''display': 'inline-block'
                                  ),
                                    
                                  # daq.Gauge(
                                  #  id='WaypontGauge1',
                                  #  showCurrentValue=True,
                                  #  #units="kampas",
                                  #  value=100,
                                  #  size=180,
                                  #  color="#273b7d",
                                  #  label='Waypoint heading',
                                  #  max=360,
                                  #  min=0,
                                  #  style={'width': '30%', 'display': 'inline-block'}
                                  #  )
                                  
                                 ], style={'padding':5, 'display': 'flex', 'flexDirection': 'row', 'border': '1px solid white','width': '30%'}
                                ), # 'gap':'10px' , 'flex': 1


                            html.Div([
                                  
                                      daq.BooleanSwitch(
                                          id='Manual_mode', 
                                          on=False,
                                          color="#55eb63",
                                          label="Manual mode",
                                          labelPosition="top"
                                          ),
                                      daq.Joystick(
                                          id='joystick-1',
                                          size=150,
                                          label="Manual controll",
                                          labelPosition="top",
                                          angle=0
                                          ),
                                      html.Button(
                                                  'START', id='start',  n_clicks=0, 
                                                  style={'backgroundColor':'#5bda74', 'color':'white','font-size':'20px','height':'50px','width':'100px'} 
                                                  ),
                                      html.Button(
                                                  'PAUSE', id='pause', n_clicks=0, 
                                                  style={'backgroundColor':'#f5cb52', 'color':'white','font-size':'20px','height':'50px','width':'100px'} 
                                                  ),
                                      html.Button(
                                                  'STOP', id='stop', n_clicks=0, 
                                                  style={'backgroundColor':'#f45826', 'color':'white','font-size':'20px','height':'50px','width':'100px'} 
                                                  ),
                                      html.Button(
                                                  'Load', id='load', n_clicks=0, 
                                                  style={'backgroundColor':'#6699CC', 'color':'white','font-size':'20px','height':'50px','width':'100px'} 
                                                  ),

                                      daq.BooleanSwitch(
                                          id='CuttMotor', 
                                          on=False,
                                          color="#55eb63",
                                          label="Cutt Motor",
                                          labelPosition="top"
                                          ),
                                      html.P(id='temp'),
                                  

                                    ], style={'display': 'flex', 'padding': 10,  'gap':'20px','border': '1px solid white'} 
                                  ),

                            ], style={'display': 'flex', 'flexDirection': 'row', 'height': '200px', 'border': '1px solid white'}  # 
                           ),
                     
                     #---------------
                     html.Div([
                               dcc.Dropdown(
                                               id="dropdown",
                                               options=[{"label": x, "value": x} for x in files],
                                               value=files[0],
                                               style={'left': '5px','top': '5px','width':'350px'}
                                               
                                               ),
                               html.Div(id='Waypindex'),
                               html.Div(id="folder_files"),
                              
                               html.Button('Step >', id='btn1'),
                               html.Button('Step <', id='btn2'),
                               html.Button('Locked', id='btn3'),
                              
                               html.Div(id='body-div')
                              ], style={'display': 'flex', 'flexDirection': 'row','padding-top': '5px','gap':'40px'}
                             ),

                     html.Br(),
                     html.Div(id='my-output'),
                    
                      ]),
                    dcc.Tab(label='Parametrs', style={'padding': '0','line-height': tab_height},selected_style={'padding': '0','line-height': tab_height},
                     children=[
                       html.Div([
                              html.Button('Get parametrai', id='btn4'),
                              html.Br(),
                              "Input1: ",
                              dcc.Input(id='inpt1',value=self.shared3.p1, type='number', min=-999999, max=999999, step=1, persistence = True, debounce=True),
                              html.Br(),
                              "Input2: ",
                              dcc.Input(id='inpt2',value=self.shared3.p2, type='number', min=-999999, max=999999, step=1, persistence = True, debounce=True),
                              html.Br(),
                              "Input3: ",
                              dcc.Input(id='inpt3',value=self.shared3.p3, type='number', min=-999999, max=999999, step=1, persistence = True, debounce=True),
                              html.Br(),
                              html.Div(id="input-out"),
                              ]),
                        html.Br(),
                         #-------------------------------
                        dash_table.DataTable(
                            id='live_table',
                            #data=[{'id':'0%', 'value':0}],
                            #columns=[{"name": i, "id": i} for i in self.tmp1[0] ],   # stulpelio name = id
                            
                            columns=[{ 'name':'Parametras', 'id':'parametras'}, {'name':'Verte', 'id':'verte'}],    # list of dict stulpeliams
                            data=[{'parametras':i, 'verte':m} for i, m in self.tmp1[0].items()],
                            editable=False,              
                            page_current=0,             
                            page_size=20,              
                            style_cell={'textAlign': 'left'},
                            style_data={'whiteSpace': 'normal','height': 'auto'},
                            fill_width=False
                            
                         ),   
                       #------------------------
                    ]),
            
                    dcc.Tab(label='Historian data', style={'padding': '0','line-height': tab_height},selected_style={'padding': '0','line-height': tab_height},
                        children=[
                            dcc.Graph(id='Hist-Graph', animate=False, style={'width': '100%', 'height': '500px'} ),   # 200vh,  70vh
                            "Display log items : ",
                            html.Div(id='slider-output'),
                            html.Div([
                                 dcc.Slider(100, 5000, 500, value=500, id='GraphSlider1')],
                                 style = {'width': '700px', 'height': '20px'}
                                  ),
                           ]),

                     
                    ]),
                 
                 html.Br(),
                     dcc.Interval(
                                 id='interval_component',
                                 interval=1000, # 1000 ms refreshs
                                 n_intervals=0, # auto increment
                                )

                ], style={ 'font-family': 'Tahoma'} )

       #------------------ History Graph --------------
        @app.callback(
                  Output('Hist-Graph','figure'),               #extendData
                  [Input('interval_component','n_intervals')]  
                 )

        def update_graph(n):
            x, y, q, yaw, speedV, speedW, tindex, disttotarg, thetaE, thetaD   = self.getlogdata()
     
            x_data=list(range(0,len(yaw),1))
            
            # Create the graph trace
            trace = [go.Scatter(
                                y=yaw,
                                x=x_data,
                                mode="lines+markers",
                                name="Yaw (rad)",
                                line={"color": "rgb(0, 255, 0)"},
                                marker={"color": "rgb(0, 255, 0)", "size": 3},
                                ),
                     go.Scatter(
                                y=speedV,
                                x=x_data,
                                mode="lines+markers",
                                name="SpeedV (m/s)",
                                line={"color": "rgb(255, 0, 0)"},
                                marker={"color": "rgb(255, 0, 0)", "size": 3},
                                ),
                     go.Scatter(
                                y=speedW,
                                x=x_data,
                                mode="lines+markers",
                                name="SpeedW (rad)",
                                line={"color": "rgb(0, 0, 255)"},
                                marker={"color": "rgb(0, 0, 255)", "size": 3},
                                ),
                     go.Scatter(
                                y=tindex,
                                x=x_data,
                                mode="lines+markers",
                                name="Target Idx",
                                line={"color": "rgb(50, 141, 168)"},
                                marker={"color": "rgb(50, 141, 168)", "size": 3},
                                ),
                     go.Scatter(
                                y=disttotarg,
                                x=x_data,
                                mode="lines+markers",
                                name="Dist to Target",
                                line={"color": "rgb(125, 30, 166)"},
                                marker={"color": "rgb(125, 30, 166)", "size": 3},
                                ),
                     go.Scatter(
                                y=thetaE,
                                x=x_data,
                                mode="lines+markers",
                                name="Theta_e",
                                line={"color": "rgb(237, 131, 9)"},
                                marker={"color": "rgb(237, 131, 9)", "size": 3},
                                ),
                     go.Scatter(
                                y=thetaD,
                                x=x_data,
                                mode="lines+markers",
                                name="Theta_d",
                                line={"color": "rgb(247, 190, 2)"},
                                marker={"color": "rgb(247, 190, 2)", "size": 3},
                                ),
                     ] 
                     
            # Create the graph layout
            layout = go.Layout(
                               #autosize=True, 
                               
                               title="Historian data:",

                               #grid = {'rows': 1, 'columns': 1, 'pattern': "independent"},
                               #xaxis=dict(range=[min(x_data), max(x_data)]),    # To show dynamicly more data
                               #yaxis=dict(range=[min(y_data), max(y_data)]),

                               xaxis= {'automargin': True, 'gridcolor': '#919194' , 'linecolor': 'white'},
                               yaxis= {'automargin': True, 'gridcolor': '#919194' , 'linecolor': 'white'},

                             )
            
            # Return the graph figure
            return {"data": trace, "layout": layout}     
            #return  [{'x':i, 'y': m, 'type': 'Scatter', 'name': 'test'} ]      #dict(x=[i], y=[m]), [0], 10

        # ---- History select slider

        @app.callback(
                       Output('slider-output', 'children'),
                       [Input('GraphSlider1', 'value')])

        def update_output(value):
               self.DisplayRecords=value
               return 'Display last Log records: "{}"'.format(value)   

        # ----- Waypoint file select -----

        @app.callback(
                       Output("Wmap", "data"), 
                       [Input("dropdown", "value")],
                       prevent_initial_call=True
                      )
        def Selectfile(file_selected):
           
            #file_list = html.Ul([html.Li(file) for file in file_names])

            self.shared3.WaypointSelected="Waypoints/"+file_selected

            with open(self.shared3.WaypointSelected, 'r') as f:
               self.geojson = json.load(f)

            self.shared3.NewSelectedWaypoint=True     # Update in main

            return self.geojson

        #-------- Buttons ----    
        @app.callback(  
                      Output('temp', 'value'),
                      [Input('start', 'n_clicks')],
                      prevent_initial_call=True
                     ) 
        def start_1(n_clicks):
            self.shared3.start =True
            self.shared3.stop =False
            #print("Start")
            return 1

        @app.callback(  
                      Output('temp', 'value2'),
                      [Input('stop', 'n_clicks')],
                      prevent_initial_call=True
                     ) 
        def stop_1(n):
            self.shared3.start =False 
            self.shared3.stop =True
            #print("Stop")
            return 1

        @app.callback(  
                      Output('temp', 'value6'),
                      [Input('pause', 'n_clicks')],
                      prevent_initial_call=True
                     ) 
        def pause_1(n):

            self.shared3.pause = not self.shared3.pause
            
            #print("Pause:", self.shared3.pause)
            return 1    

        #--------- Joystick -----
        @app.callback(
                       Output('temp', 'value3'),
                      [Input('joystick-1', 'angle'),
                       Input('joystick-1', 'force')]
                      )
        def update_output(angle,force):
            self.shared3.Jangle=angle
            self.shared3.Jforce=force
            return 1

        @app.callback(
                      Output('temp', 'value4'),
                     [Input('Manual_mode', 'on')]
                     )
        def update_output(on):
           self.shared3.manual=on
           return 1

        @app.callback(
                      Output('temp', 'value5'),
                     [Input('CuttMotor', 'on')]
                     )
        def update_output(on):
           self.shared3.CuttMot=on
           return 1


        @callback(  
                    Output('slider1', 'value'),
                    Input('btn1', 'n_clicks'),
                    #Input('btn2', 'n_clicks'),
                    #Input('btn3', 'n_clicks')
                    ) 

        def change_text(n_clicks):
            return ["n_clicks is " + str(n_clicks), "n_clicks is " + str(n_clicks)]

        #---------- Parametrai ---------
        @app.callback(  
                      Output("input-out", "children"),
                      [Input("inpt1", "value"),
                      Input("inpt2", "value"),
                      Input("inpt3", "value")]
                    )
        def param_input_render(p1, p2, p3):
            self.shared3.p1 =p1
            self.shared3.p2 =p2
            self.shared3.p3 =p3
            return "inpt1: {}, inpt2: {}, inpt3: {}".format(self.shared3.p1, self.shared3.p2, self.shared3.p3)


        #---------------Live data table-----------------
        @app.callback(
                       Output('live_table','data'),
                       [Input('interval_component','n_intervals')]
                     )  

        def update_table(n):        
           #self.tmp1=[shared.return_class_variables()]       
           data2=[{'parametras':i, 'verte':m} for i, m in self.tmp1[0].items()]    # lenteles refresui  
           return  data2 


        #---------- Map + Gauges ---------
        @app.callback(
                      Output('position-data','position'),       # marker update
                      Output('RobototGauge1','value'),          # robot gauge1
                      Output('Waypindex','children'),
                     [Input('interval_component','n_intervals')]
                     )
        
        def update_map(m):            
           data3=[self.tmp1[0]['lat'], self.tmp1[0]['long']]                       
           #print ("lat, long ",self.tmp1[0]['lat'], self.tmp1[0]['long'] )
           data4=self.shared2.Direction

           return  data3, data4,  "Waypoint target: {}".format(self.shared3.TargetIndex)  


     
        #if __name__ == '__main__':
        print("www start")
        app.run(host="0.0.0.0", debug=False)