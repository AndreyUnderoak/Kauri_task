from pkgutil import extend_path
import serial
import matplotlib.pyplot as plt
import time

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0, parity=serial.PARITY_EVEN, rtscts=1,stopbits=serial.STOPBITS_ONE)
ser.baudrate = 9600
ser.bytesize=serial.SEVENBITS
ser.parity=serial.PARITY_NONE
ser.stopbits=serial.STOPBITS_ONE
ser.xonxoff=1
ser.rtscts=0

plt.ion()
fig = plt.figure()
ax = plt.axes(projection="3d")

while(1):
    text = ser.readline()
    try:
        _, x, _, y, _, z= text.split()
        x = str(x).replace("b'", "")
        x = str(x).replace("'", "")
        y = str(y).replace("b'", "")
        y = str(y).replace("'", "")
        z = str(z).replace("b'", "")
        z = str(z).replace("'", "")
        x,y,z = float(x), float(y), float(z)
        print(x,y,z)
        ax.plot3D(x, y, z, 'red')
        ax.scatter3D(x, y, z)
        plt.show()
        plt.pause(0.1) 
    except:
        continue