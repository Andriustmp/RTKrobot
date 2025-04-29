"""
 RTK robot mower V 1.4
 WEB user interface  x.x.x.x:8050
 2024 05 23
 author: A. Besusparis
"""

import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

import os
import json
import threading 
import random
import dash_leaflet as dl
import dash_daq as daq
from   dash import Dash, html, Output, Input 
from   dash import dash_table, dcc, callback
import plotly.graph_objs as go

files = os.listdir("Waypoints")

class robotweb(threading.Thread):

    def __init__(self,shared, shared2, shared3, shared4): 
          self.tmp1=[shared.return_class_variables()]     # display list of dict
          self.shared2 = shared2
          self.shared3 = shared3
          self.shared4 = shared4
          self.pozicija=[55.2295, 25.7254]                # map draw center     
          LogData = self.shared4 
          self.Graph1index=1  
          self.DisplayRecords=250
          self.geojson=[]

          with open(self.shared3.WaypointSelected, 'r') as f:
               self.geojson = json.load(f)
          

    def ListFiles(self):
            file_names = os.listdir("Waypoint")
            return file_names      
                
    def getlogdata(self):

        x          = []
        y          = []
        Q          = []
        yaw        = []
        speedV     = []
        speedW     = []
        tindex     = []
        disttotarg = []
        thetaE     = []
        thetaD     = []
        latErr     = []

        logitems=len(self.shared4.log1)
        
        if (logitems>self.DisplayRecords):       # how many historical records to show in the graph
           start=logitems-self.DisplayRecords
        else: start=0   

        for index in range(start, logitems ):
           # x.append(self.shared4.log1[index]['lat'])
           # y.append(self.shared4.log1[index]['long'])
           # Q.append(self.shared4.log1[index]['Q'])
            yaw.append(self.shared4.log1[index]['yaw'])
            speedV.append(self.shared4.log1[index]['v'])
            speedW.append(self.shared4.log1[index]['w'])
            tindex.append(1+self.shared4.log1[index]['Tidx']/10)       # scale to fit graph
            disttotarg.append(self.shared4.log1[index]['DistToTarg'])
            thetaE.append(self.shared4.log1[index]['Theta_e'])
            thetaD.append(self.shared4.log1[index]['Theta_d'])
            latErr.append(self.shared4.log1[index]['latErr'])

        return  x, y, Q, yaw, speedV, speedW, tindex, disttotarg, thetaE, thetaD, latErr   

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
                                       dl.GeoJSON(url="/assets/MapTest2.geojson", id="states"),
                                       dl.GeoJSON(url=self.Waypoint_selected, id='Waiponts_map', options={"style":{"color":" #df8400","weight":"1"}}),  
                                      ], 
                                      center=(55.2295, 25.7254), zoom=20, style={'height': '60vh'}  
                            ), 
                     #---------------
                     html.Div([
                         html.Div([   
                                  daq.Gauge(
                                             id='RobototGauge1',
                                             showCurrentValue=True,
                                             size=135,
                                             color="#273b7d",
                                             value=self.shared2.Direction,
                                             label='Robot heading (deg)',
                                             max=360,
                                             min=0,
                                             style={'left': '10px'}      # 'width': '40%''display': 'inline-block'
                                            ),
                                   html.Div([
                                              html.H1('Robot status:', style={'padding': 0,'fontSize': 12}),
                                              html.Div(id='Robot_status'),
                                              html.H1(id='SafeZone',  style={'padding': 0,'fontSize': 12}),
                                             ], style={'display': 'flex', 'flexDirection': 'column','padding': '30px','gap':'5px',}
                                            ), 
      
                                  ], style={'display': 'flex', 'flexDirection': 'row','padding': '10px','gap':'2px','border-style': 'solid','border-radius': '15px', 'border-color':'#6495ED','width': '350px'} 
                                 ), 

                                        html.Div([
                                                  daq.BooleanSwitch(
                                                                      id='Manual_mode', 
                                                                      on=False,
                                                                      color="#55eb63",
                                                                      label="Manual mode",
                                                                      labelPosition="top",
                                                                      persistence = True,
                                                                    ),
                                                  daq.Joystick(
                                                                id='joystick-1',
                                                                size=150,
                                                                #label="Manual controll",
                                                                labelPosition="top",
                                                                angle=0
                                                              ),
                                                  ], style={'display': 'flex', 'flexDirection': 'row','padding': '10px','gap':'20px','border-style': 'solid','border-radius': '15px', 'border-color':'#6495ED',} 
                                                  ),

                                        html.Div([ 
                                                   html.Div( ['Automatic mode'], style={'text-align': 'center'} ),
                                                   html.Div([
                                                               
                                                              html.Button(
                                                                          'START', id='start',  n_clicks=0, 
                                                                          style={'backgroundColor':'#006400', 'color':'white','font-size':'20px','height':'50px','width':'100px'} 
                                                                          ),
                                                              html.Button(
                                                                          'PAUSE', id='pause', n_clicks=0, type='button',
                                                                          style={'backgroundColor':'#DAA520', 'color':'white','font-size':'20px','height':'50px','width':'100px'} 
                                                                          ),
                                                              html.Button(
                                                                          'STOP', id='stop', n_clicks=0, 
                                                                          style={'backgroundColor':'#DC143C', 'color':'white','font-size':'20px','height':'50px','width':'100px'} 
                                                                          ),

                                                              ], style={'display': 'flex', 'flexDirection': 'row', 'padding': '20px','justify-content': 'center', 'gap':'20px'}
                                                              ),
                                                    html.Div([
                                                              html.Div(id='Waypindex',       style={'padding': 0,'fontSize': 15}),
                                                              html.Div(id='last_Waypindex',  style={'padding': 0,'fontSize': 15}),
                                                             ], style={'display': 'flex', 'flexDirection': 'row', 'padding': '10px','justify-content': 'center', 'gap':'20px'}
                                                            ),
                                                    ], style={'display': 'flex', 'flexDirection': 'column','padding': '10px','gap':'20px','border-style': 'solid','border-radius': '15px', 'border-color':'#6495ED',}   
                                                   ),

                                           html.Div([
                                                      daq.BooleanSwitch(
                                                                          id='CuttMotor', 
                                                                          on=False,
                                                                          color="#55eb63",
                                                                          label="Cutt Motor",
                                                                          labelPosition="top",
                                                                          persistence = True,
                                                                       ),
                                                      
                                                      html.Div([ 
                                                                html.Div( ['Speed m/s:'],style={'text-align': 'center'} ),
                                                                dcc.Input(id='input_speed',  
                                                                          value=self.shared3.TargetSpeed, 
                                                                          type='number', min=0.2, max=1.3, step=0.05, 
                                                                          persistence = True, 
                                                                          debounce=False
                                                                          ),
                                                                ], style={'display': 'flex', 'flexDirection': 'column', 'padding': '5', 'justify-content': 'center', 'gap':'1px'}
                                                                ),
                                                    ], style={'display': 'flex', 'flexDirection': 'column','gap':'20px','padding': '10px','border-style': 'solid','border-radius': '15px', 'border-color':'#6495ED',}   
                                                   ),
                                  

                            ], style={'display': 'flex', 'flexDirection': 'row', 'padding': '5px','gap':'2px','height': '200px', 'border': '0px solid green'}  # 
                           ),
                     
                     #---------------
                     html.P(id='temp'),
                     html.Div([
                               dcc.Dropdown(
                                              id="dropdown",
                                              options=[{"label": x, "value": x} for x in files],
                                              #value=files[0],
                                              style={'left': '5px','top': '5px','width':'370px'},
                                              #persistence = True 
                                            ),
                               dcc.ConfirmDialogProvider(
                                                         children=html.Button('Power OFF ',style={'color':'red','font-size':'15px','height':'40px','width':'100px'}),
                                                         id='PowerOFF',
                                                         message='Power off robot ?',
                                                         ),
                               html.Div(id='off-output'),
                              
                              ], style={'display': 'flex', 'flexDirection': 'row', 'padding-top': '0px','gap':'100px'}
                             ),

                     html.Br(),
                     html.Div(id='my-output'),
                    
                      ]),
                    dcc.Tab(label='Parameters', style={'padding': '0','line-height': tab_height},selected_style={'padding': '0','line-height': tab_height},
                     children=[
                       html.Div([
                                  html.Button('Get parametrai', id='btn4'),
                                  #html.Br(),
                                  html.P(),
                                  "theta_d (lateral Error gain) : ",
                                  dcc.Input(id='input_theta_d',value=self.shared3.k_td , type='number', min=0, max=3, step=0.01, persistence = False, debounce=False),
                                  html.P(),
                                  "theta_e (yaw Error gain)     : ",
                                  dcc.Input(id='input_theta_e',value=self.shared3.k_te , type='number', min=0, max=3, step=0.01, persistence = False, debounce=False),
                                  html.P(),
                                  "Kstear (steering overshoot  ): ",
                                  dcc.Input(id='input_Kstear',value=self.shared3.Kstear, type='number', min=0, max=3, step=0.01, persistence = False, debounce=False),
                                  html.P(),
                                  html.Div(id="input-out"),
                               ]),

                        html.Br(),
                        #-------------------------
                        dash_table.DataTable(
                                             id='live_table',
                                             columns=[{ 'name':'Parameter', 'id':'parametras'}, {'name':'Value', 'id':'verte'}],    
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
                                   html.Div([
                                             dcc.Graph(id='Hist-Graph', animate=False, style={'width': '100%', 'height': '320px'} ),   # 200vh,  70vh
                                             dcc.Graph(id='Hist-Graph2', animate=False, style={'width': '100%', 'height': '320px'} ),   # 200vh,  70vh
                                            ], style={'display': 'flex', 'flexDirection': 'column', 'padding-top': '5px','gap':'5px'}
                                            ),
                                  #"Display log items : ",
                                  html.Div(id='slider-output'),
                                  html.Br(),
                                  html.Div([
                                            dcc.Slider(250, 2000, 250, value=250, id='GraphSlider1',marks=None, dots=True)],
                                            style = {'width': '400px', 'height': '20px'}
                                          ),
                                ]),

                     
                 ]),
                 
                 html.Br(),
                     dcc.Interval(
                                 id='interval_component',
                                 interval=1000,           # 1000 ms refresh
                                 n_intervals=0,           # auto increment
                                )

                ], style={ 'font-family': 'Tahoma'} )

       #------------------ History Graph --------------
        @app.callback(
                      Output('Hist-Graph','figure'),                #extendData
                      Output('Hist-Graph2','figure'),
                      [Input('interval_component','n_intervals')]  
                     )

        def update_graph(n):
            x, y, q, yaw, speedV, speedW, tindex, disttotarg, thetaE, thetaD, latErr   = self.getlogdata()
            
            x_data=list(range(0,len(yaw),1))

            # Create the graph1 trace
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

            trace2 = [go.Scatter(
                                y=latErr,
                                x=x_data,
                                mode="lines+markers",
                                name="Lat_Error (cm)",
                                line={"color": "rgb(0, 255, 0)"},
                                marker={"color": "rgb(0, 255, 0)", "size": 3},
                                ),
                      ]                    
                     
            # Create the graph layout
            layout = go.Layout(
                               autosize=True, 
                               margin=dict(l=5, r=5, t=15, b=5),
                               showlegend=True,
                               xaxis= {'automargin': True, 'gridcolor': '#919194' , 'linecolor': 'white'},
                               yaxis= {'automargin': True, 'gridcolor': '#919194' , 'linecolor': 'white'},

                             )
            layout2 = go.Layout(
                               
                               autosize=True,
                               margin=dict(l=5, r=5, t=15, b=5),
                               showlegend=True,
                               #title="lateral Error(cm):",
                               xaxis= {'automargin': True, 'gridcolor': '#919194' , 'linecolor': 'white'},
                               yaxis= {'automargin': True, 'gridcolor': '#919194' , 'linecolor': 'white'},

                              )
            
            # Return the graph figure
            return {"data": trace, "layout": layout} ,  {"data": trace2, "layout": layout2}          
        # ------------ History select slider -------
        @app.callback(
                       Output('slider-output', 'children'),
                       [Input('GraphSlider1', 'value')]
                     )

        def update_output(value):
               self.DisplayRecords=value
               return 'Display last Log records: "{}"'.format(value)   

        # ------------ Waypoint file select --------

        @app.callback(
                       Output("Waiponts_map", "data"), 
                       [Input("dropdown", "value")],
                       prevent_initial_call=True
                      )

        def Selectfile(file_selected):
            self.shared3.WaypointSelected="Waypoints/"+file_selected
            with open(self.shared3.WaypointSelected, 'r') as f:
                 self.geojson = json.load(f)

            self.shared3.NewSelectedWaypoint=True     # Update in main

            return self.geojson

        #------------------- Buttons ----------------  
        @app.callback(  
                      Output('temp', 'value'),
                      [Input('start', 'n_clicks')],
                      prevent_initial_call=True
                     ) 
        def start_1(n_clicks):
            self.shared3.start = True
            self.shared3.stop  = False
            return 1

        @app.callback(  
                      Output('temp', 'value2'),
                      [Input('stop', 'n_clicks')],
                      prevent_initial_call=True
                     ) 
        def stop_1(n):
            self.shared3.start = False 
            self.shared3.stop  = True
            return 1

        pause_style_normal={'backgroundColor':'#DAA520', 'color':'white','font-size':'20px','height':'50px','width':'100px'}   
        pause_style_active={'backgroundColor':'#708090', 'color':'white','font-size':'20px','height':'50px','width':'100px'}  

        @app.callback(  
                      Output('temp', 'value6'),
                      Output('pause','style'),
                      [Input('pause', 'n_clicks')],
                      prevent_initial_call=True
                     ) 
        def pause_1(n):

            self.shared3.pause = not self.shared3.pause
            if (self.shared3.pause==False):
                status_style= pause_style_normal
                
            else:
                status_style= pause_style_active
                
            return 1,  status_style

        #-------- Speed value select -------
        @app.callback(  
                       Output('temp', 'value7'),
                      [Input("input_speed", 'value')],
                     )

        def speedselect(v1):
            self.shared3.TargetSpeed=v1
            return 1    

        @app.callback(
                       Output('off-output', 'children'),
                       [Input('PowerOFF', 'submit_n_clicks')],
                       prevent_initial_call=True
                      )

        def update_output(submit_n_clicks):
           if not submit_n_clicks:
              return ''

           self.shared3.shutdown =True   
           return """
                  Robot shutdown init: {} 
                  """.format(submit_n_clicks)    

        #-------------- Joystick ---------
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
        # ----------- Cutting motor ------ 
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
                  ) 

        def change_text(n_clicks):
            return ["n_clicks is " + str(n_clicks), "n_clicks is " + str(n_clicks)]

        #-------------- Parameters ---------
        @app.callback(  
                      Output("input-out", "children"),
                      [Input("input_theta_d", "value"),
                      Input("input_theta_e", "value"),
                      Input("input_Kstear", "value")]
                     )
        def param_input_render(p1, p2, p3):
            self.shared3.k_td   = p1
            self.shared3.k_te   = p2
            self.shared3.Kstear = p3
            return "inpt1: {}, inpt2: {}, inpt3: {}".format(self.shared3.k_td, self.shared3.k_te, self.shared3.Kstear)


        #---------------Live data table-----------
        @app.callback(
                      Output('live_table','data'),
                      [Input('interval_component','n_intervals')]
                     )  

        def update_table(n):              
            data2=[{'parametras':i, 'verte':m} for i, m in self.tmp1[0].items()]      # Refresh data table 
            return  data2 


        #---------- Map + Gauges ---------

        green_label_style ={'backgroundColor':'green', 'color':'white','font-size':'20px','height':'50px','width':'100px',
                            'display': 'flex','justify-content': 'center', 'align-items': 'center'}
        red_label_style   ={'backgroundColor':'red', 'color':'white','font-size':'20px','height':'50px','width':'100px',
                            'display': 'flex','justify-content': 'center', 'align-items': 'center'}

        @app.callback(
                      [Output('position-data','position'),       # marker update
                       Output('RobototGauge1','value'),          # heading gauge
                       Output('Robot_status','children'),
                       Output('Robot_status','style'),
                       Output('Waypindex','children'),
                       Output('last_Waypindex','children'),
                       Output('SafeZone','children'),
                       Output('CuttMotor', 'on'),                # update Cutter motor status
                       ],
                      [Input('interval_component','n_intervals')]
                     )

        def update_map(m):            
            data3=[self.tmp1[0]['lat'], self.tmp1[0]['long']]                       
            data4=self.shared2.Direction

            if (self.shared3.Robot_status==False):
                status_style=red_label_style
                status_label="E-STOP"
            else:
                status_style=green_label_style
                status_label="OK"   

            return  data3, data4 , status_label, status_style,  "Waypoint target: {}".format(self.shared3.TargetIndex), "Last target: {}".format(self.shared3.last_TargetIndex), "SafeZone: {}".format(self.shared3.Safe_zone), self.shared3.CuttMot    

 
        #if __name__ == '__main__':
        print("www start")
        app.run(host="0.0.0.0", debug=False)