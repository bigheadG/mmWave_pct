###################################################################################
# Parameters:
JB_UART_PORT = '/dev/tty.SLAB_USBtoUART'
JB_TILT_DEGREE = 45  
JB_RADAR_INSTALL_HEIGHT = 2.41 # meter
# for verifying (y1, z1) => (y, z); expected (1, 0) => (0.50, 0.86)
###################################################################################

#=============================================
# File Name: PCT_ex1_pyqtgraph_3d_tid.py
#
# Requirement:
# Hardware: BM201-ISK or BM501-AOP
#
# lib: pct
#
# graphics tools: pyqtgraph Version: 0.13.1
# use v8 to remove unwanted point cloud
# Plot point cloud(V6) in 2D/3D figure 
# 
# type: Raw data
# Baud Rate:  
#			real time: 921600
#
# Copy from POS:
#=============================================

import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import mkQApp ,QtCore , QtGui

import numpy as np

from mmWave import pct
import serial
from threading import Thread
from datetime import date,datetime 

st = datetime.now()
sim_startFN = 0
sim_stopFN = 0

#
#Create coordinate Axis , label , radar, grid
#
def coordInfo(gl,gview,x=None,y=None,z=None, install_height = None,fontSize = None,gx=None,gy=None,gz=None):
	#(0)create axis 
	axisitem = gl.GLAxisItem()
	axisitem.setSize(x=x,y=y,z=z)
	gview.addItem(axisitem)
	size = 10 if fontSize == None else fontSize
	xo = np.linspace(1, x, x)
	yo = np.linspace(1, y, y)
	zo = np.linspace(1, z, z)
	
	for i in range(len(xo)):
		axisX = gl.GLTextItem(pos=(xo[i], 0.0, 0.0),  text=f'{xo[i]}' if i != len(xo)-1 else 'X' ,color=(255, 127, 127, 255),font=QtGui.QFont('Helvetica', size))
		gview.addItem(axisX)
	for i in range(len(yo)):
		axisY = gl.GLTextItem(pos=( 0.0, yo[i], 0.0), text=f'{yo[i]}' if i != len(yo)-1 else 'Y' ,color=(127, 255, 127, 255),font=QtGui.QFont('Helvetica', size))
		gview.addItem(axisY)
	for i in range(len(zo)):
		axisZ = gl.GLTextItem(pos=( 0.0, 0.0, zo[i]), text=f'{zo[i]}' if i != len(zo)-1 else 'Z' ,color=(127, 127, 255, 255),font=QtGui.QFont('Helvetica', size))
		gview.addItem(axisZ)
	
	#(1)create box to represent device 
	zOffSet = install_height
	verX = 0.0625
	verY = 0.05
	verZ = 0.125
	verts = np.empty((2,3,3))
	verts[0,0,:] = [-verX, 0, verZ + zOffSet]
	verts[0,1,:] = [-verX, 0,-verZ + zOffSet]
	verts[0,2,:] = [verX,  0,-verZ + zOffSet]
	verts[1,0,:] = [-verX, 0, verZ + zOffSet]
	verts[1,1,:] = [verX,  0, verZ + zOffSet]
	verts[1,2,:] = [verX,  0, -verZ + zOffSet]
	
	#(2)create evmBox 
	evmBox = gl.GLMeshItem(vertexes=verts,smooth=False,drawEdges=True,edgeColor=pg.glColor('r'),drawFaces=False)
	gview.addItem(evmBox)
	
	#(2)create gridX,Y 
	grid = gl.GLGridItem()
	grid.setSize(x=gx,y=gy,z=gz)
	gview.addItem(grid)
	
############################################

#app = QtGui.QApplication([]) 
app = mkQApp("PCT")

#################for v6 3D plot ###########################
win3D = gl.GLViewWidget()
win3D.setWindowTitle('(w) V6:sp0 3D Chart')

# set Scatter plot
pos = np.zeros((100,3))
color = [1.0, 0.0, 0.0, 1.0]
sp0 = gl.GLScatterPlotItem(pos=pos,color=color,size = 8.0)
#sp2 = gl.GLScatterPlotItem(pos=pos,color=color,size = 20.0)


win3D.addItem(sp0)
#win3D.addItem(sp2)

win3D.show()
coordInfo(gl,win3D,x=6,y=6,z=4,install_height= JB_RADAR_INSTALL_HEIGHT,fontSize=12,gx = 50,gy=50,gz= 50)

###################################################################
#
#use USB-UART
#port = serial.Serial("/dev/ttyUSB0",baudrate = 921600, timeout = 0.5)
#
#for Jetson nano UART port
#port = serial.Serial("/dev/ttyTHS1",baudrate = 921600, timeout = 0.5) 
#
#for pi 4 UART port
#port = serial.Serial("/dev/ttyS0",baudrate = 921600, timeout = 0.5)
#
#Drone Object Detect Radar initial 
#port = serial.Serial("/dev/tty.usbmodemGY0052534",baudrate = 921600, timeout = 0.5)
#port = serial.Serial("/dev/tty.usbmodem14103",baudrate = 115200 , timeout = 0.5)  
#port = serial.Serial("/dev/tty.usbmodemGY0050674",baudrate = 921600, timeout = 0.5) 
#port = serial.Serial("/dev/tty.SLAB_USBtoUART5",baudrate = 921600, timeout = 0.5)   

#for NUC ubuntu 
#port = serial.Serial("/dev/ttyACM1",baudrate = 921600, timeout = 0.5)

#for INTEL NUC 

port = serial.Serial(JB_UART_PORT,baudrate = 921600, timeout = 0.5)   

radar = pct.Pct(port, tiltAngle= JB_TILT_DEGREE, height = JB_RADAR_INSTALL_HEIGHT)

v6len = 0

#=====================xy scatter========================
win2D = pg.GraphicsLayoutWidget() #pg.GraphicsWindow()
win2D.resize(600,600)
#pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'y')
 
win2D.setWindowTitle('(w0) xy Points Cloud v6:sp1')
w0 = win2D.addPlot()
w0.setRange(xRange=[-6,6],yRange= [0,6])
w0.setLabel('bottom', 'V6 PointCloud', 'meter')
w0.setLabel('left', 'Y', 'meter')
w0.invertX(True)
w0.invertY(True)
w0.showGrid(x = True, y = True, alpha = 0.7)    
#w0.invertY(True)
 
#for v6
sp1 = pg.ScatterPlotItem(size = 3, pen=pg.mkPen('w'), pxMode=True) #pg.ScatterPlotItem(pxMode=True)   ## Set pxMode=False to allow spots to transform with the view
#for v7
#sp3 = pg.ScatterPlotItem(size = 10, pen=pg.mkPen('w'), pxMode=True) #pg.ScatterPlotItem(pxMode=True)   ## Set pxMode=False to allow spots to transform with the view

w0.addItem(sp1) 
#w0.addItem(sp3)
win2D.show()

############# V6 points cloud hit counting ##############
# parameter
hc_xRange = 100
hc_yRange = 100
v6pcA  = np.zeros(hc_xRange-1)
pctA = np.linspace(0,hc_xRange,hc_xRange)

winP = pg.GraphicsLayoutWidget()
winP.setWindowTitle('V6 hit count')
p5 = winP.addPlot(title="v6 hit count") 
p5.setRange(yRange=[0,hc_yRange])
bar = p5.plot(pen='y')
# add item
winP.addItem(p5)
winP.show()

sensorA = []

colorSet = [
    (1.0, 0.0, 0.0, 1.0),   # red
    (0.0, 1.0, 0.0, 1.0),   # green
    (0.0, 0.0, 1.0, 1.0),   # blue
    (1.0, 1.0, 0.0, 1.0),   # yellow
    (0.0, 1.0, 1.0, 1.0),   # cyan
    (1.0, 0.0, 1.0, 1.0),   # magenta
    (0.0, 0.0, 0.0, 1.0),   # black
    (1.0, 1.0, 1.0, 1.0)    # white
]

colors = ['r', 'g','b', 'y', 'c', 'm', 'w']

def update():
	global color,sensorA,sp0,sp1,colors,colorSet
	global pctA,v6pcA,bar
	
	#v6 hit counts
	bar.setData(pctA,v6pcA, stepMode="center", fillLevel=0, fillOutline=True, brush=(0,0,255,150))

	#(1)for 3D/2D points cloud v6
	try:
		if sensorA.shape[0] != 0 and sensorA.shape[1] == 10:
			#(1.1)for 2D v6
			#[(sx,sy,sz,ran,elv,azi,dop,snr,fn)...]
			#   0  1  2  3  4   5    6   7   8
			#sp1.setData(x=sensorA[:,0],y=sensorA[:,2], pen = 'g', symbol='s')

			spots1  = [{'pos': [sensorA[i][0],sensorA[i][2]], 'data': 1, 'brush': colors[sensorA[i][9]%8], 'symbol': 's', 'size': 5 } for i in range(len(sensorA))]
			sp1.setData(spots1)
			
			#(1.2)for 3D v6
			trA = sensorA
			trA = trA[:,(0,2,1)] # x, z, y(height)
			colorA = [colorSet[sensorA[i][9]%8] for i in range(len(sensorA))]
			sp0.setData(pos=trA,color= np.array(colorA))

		else:
			sp0.setData()
			sp1.setData()
	except:
		pass
			


t = QtCore.QTimer()
t.timeout.connect(update)
t.start(150)

def showData(dck,v6i,v7i,v8i):
	if dck:
		v6len = len(v6i)
		v7len = len(v7i)
		v8len = len(v8i)
		print("Sensor Data: [v6,v7,v8]:[{:d},{:d},{:d}]".format(v6len,v7len,v8len))
		
		if v6len > 0:
			print("\n--------v6-----------fn:{:} len({:})".format(fn,v6len))
			print(v6i)
		if v7len > 0:
			print("\n--------v7-----------fn:{:} len({:})".format(fn,v7len))
			print(v7i)
		if v8len > 0:
			print("\n--------v8-----------fn:{:} len({:})".format(fn,v8len))
			print(v8i)
		
#[(sx,sy,sz,ran,elv,azi,dop,snr,fn,tid)...]
QUE_LEN = 6
QUE = [[(0,0,0,0,0,0,0,0,0,0,0)]] * QUE_LEN 
q_ptr = 0
def QUE_INSERT(data):
    global q_ptr,QUE_LEN
    QUE[q_ptr] = data
    q_ptr += 1
    q_ptr %= QUE_LEN  


fn = 0
prev_fn = 0
v6_prev = []

def radarExec():
	global v6len,v7len,v8len,prev_fn,fn,color,flag,uFlag,sim_stopFN,fn,sensorA,v6_prev
	v6 = []
	flag = True
	(dck,v6,v7,v8)  = radar.tlvRead(False)
	 
	hdr = radar.getHeader()
	fn = hdr.frameNumber
	showData(dck,v6,v7,v8)
	
	if fn != prev_fn:
		prev_fn = fn
		v6len = len(v6)
		print('fn: {:} v6  v6-points:{:} v8={:} set:{:}'.format(fn,v6len,len(v8),set(v8))) 
		v6pcA[:-1] = v6pcA[1:]
		v6pcA[-1] = len(v6)
		
		#v6 [(sx,sy,sz,ran,elv,azi,dop,snr,fn)...]
		v6i = [] # [(sx,sy,sz,ran,elv,azi,dop,snr,fn,tid)...]
		if len(v8) == len(v6_prev) and len(v8) > 0:
			for i in range(len(v8)):
				v = tuple(v6_prev[i]) + (v8[i],)
				v6i.append(v)
		
		QUE_INSERT(v6i)
		
		v6_prev = np.array(v6)
		dBuf = []
		for q in QUE:
			for i in q: # data: [sx,sy,sz,ran,elv,azi,dop,snr,fn] 
				if i[9] < 252:
					dBuf.append(i)
		
		sensorA = np.array(dBuf,dtype=object)

		 
def uartThread(name):
	port.flushInput()
	while True:
		radarExec()
		port.flushInput()
		
					
thread1 = Thread(target = uartThread, args =("UART",))
thread1.setDaemon(True)
thread1.start()


if __name__ == '__main__':
    pg.exec()
 
