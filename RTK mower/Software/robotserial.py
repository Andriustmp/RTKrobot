
"""
 RTK robot mower V 1.4
 Serial coms with robot MCU 
 2024 05 23
 author: A. Besusparis
""" 

import serial
import time
import threading 

class rserial(threading.Thread): 

   def __init__(self, shared, shared2, shared3):
      self.ser = serial.Serial(port='/dev/ttyUSB0', baudrate=19200, bytesize=8, parity='N', stopbits=1, timeout=1)
      #print("RX start")
      self.shared = shared
      self.shared2 = shared2
      self.shared3 = shared3
      
   def RXdata (self,lock):
           while (True):
               
               # Check if incoming bytes are waiting to be read from the serial input 
               # buffer.
               # NB: for PySerial v3.0 or later, use property `in_waiting` instead of
               # function `inWaiting()` below!
               if (self.ser.inWaiting() > 0):
                   # read the bytes 
                   self.data_str = self.ser.read(self.ser.inWaiting())   # dec format    #.decode('ascii')   # .hex()  # .decode('utf-8').rstrip()   
                   # print the incoming string without putting a new-line
                   # ('\n') automatically after every print()
                   #print(data_str, end='') 
                   #print(self.data_str)
                   self.decode(lock)
                   # Optional, but recommended: sleep 10 ms (0.01 sec) once per loop to let 
                   # other threads on your PC run during this time.
               #self.ser.write('test1'.encode('utf-8'))
               time.sleep(0.01)
               self.TXdata(lock)

   # 7 byte to MCU 
   def TXdata(self, lock):                       # todo CRC
        if (self.shared2.Transmit==1):
                txarray = bytearray(7)    
               
                txarray[0]=0x53   # S
                txarray[1]=ord(self.shared2.LmotorFR)
                txarray[2]=self.shared2.Lspeed
                txarray[3]=ord(self.shared2.RmotorFR)
                txarray[4]=self.shared2.Rspeed
                txarray[5]=self.shared2.Cutter_RunStop
                txarray[6]=0x45   # E 

                self.ser.write(txarray)
               # print(hex(txarray))
                lock.acquire()
                self.shared2.Transmit=0   
                time.sleep(0.02)
                lock.release()

                
   # 17 byte packet from MCU: ( "T" + EmergData+BatUH + BatUL + BatIH + BatIL + CompasH + Compasl
   # + LWheelcnt(4byte) + RWheelcnt(4byte) + "E" )             
   def decode (self,lock):
           Estop2=0
           Ebutton2=0
           Esensor2=0
           BatU=0
           Dir2=0
           cnt1=0
           cnt2=0

           if (len(self.data_str) ==17) and (self.data_str[0]==84)and (self.data_str[16]==69):  # dec

               if (self.data_str[1] & 0b1):   Estop2=1
               if (self.data_str[1] & 0b10):  Ebutton2=1
               if (self.data_str[1] & 0b100): Esensor2=1

               BatU=self.data_str[2]<<4
               BatU=BatU | self.data_str[3]
               if (BatU <0) or (BatU>500): BatU=0
               BatU=BatU/10

               Dir2=self.data_str[6]<<4
               Dir2=Dir2 | self.data_str[7]
               if (Dir2 <0) or (Dir2>360): Dir2=-1
               #print("Dir2:",Dir2)
               
               cnt1=self.data_str[8]<<24
               cnt1=cnt1 | self.data_str[9]<<16
               cnt1=cnt1 | self.data_str[10]<<8
               cnt1=cnt1 | self.data_str[11]
               #print("cnt1:",cnt1)
               
               cnt2=self.data_str[12]<<24
               cnt2=cnt2 | self.data_str[13]<<16
               cnt2=cnt2 | self.data_str[14]<<8
               cnt2=cnt2 | self.data_str[15]
               #print("cnt2:",cnt2)

               lock.acquire()
               self.shared.Recived=1
               self.shared.PacketCnt+=1
               self.shared.Estop=Estop2
               self.shared.Ebutton=Ebutton2
               self.shared.Esensor=Esensor2
               self.shared.BatVoltage=BatU
               self.shared.Direction=self.AngleCorection(Dir2)
               self.shared.LWheelCnt=cnt1
               self.shared.RWheelCnt=cnt2
               lock.release()

           else: 
                print("ERROR: Wrong packet from MCU")  

   def  AngleCorection(self, angle):
        angle_cor = (angle+self.shared3.compasss_corection)%360
        return (angle_cor)

                  

               
               