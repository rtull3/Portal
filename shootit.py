import serial #using the pyserial module open a serial connection
import time	

trigger=serial.Serial('/dev/ttyUSB0',115200)
#trigger.write("#0 P100 U2500\r")

trigger.open()
if trigger.isOpen():
   trigger.write("#0 P2500 U500\r")
   print('tried it')
   time.sleep(.3) #wait for microcontroller act
   trigger.write("#0 P1350 U500\r")
    #reads up to X bytes/microcontroller responds to pull command


trigger.close() #close serial connection at the end of the program
