from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QGraphicsScene, QGraphicsView, QGraphicsItemGroup, QFrame
from PyQt5.QtCore import QThread, QObject, pyqtSignal, Qt
from PyQt5.QtGui import QPainter, QBrush, QPen, QColor

import sys
import time
import serial
import utm
import numpy as np
import math
import RPi.GPIO as GPIO

class NMEAworker(QObject):	
	
	sig = pyqtSignal(list)
	prt = '/dev/ttyACM0'
	bd =115200
	ser = serial.Serial(prt, bd)
			
	def __init__(self, a=None, b=None, c=None, d=None, e=None, f=None, g=None):
		super(NMEAworker, self).__init__()
		self.znnumdef = a
		self.znletdef = b
		self.Eoffset = c
		self.Noffset = d
		self.isrunning = False
		self.ispaused = True
		self.ABa = e
		self.ABb = f
		self.ABc = g
		self.lightbarsensetivity = 0.1
		self.lst = [0,0,0,0,0,0,0,0,0,0,0]
	
	def NMEAtoUTM(self):
		self.isrunning = True
		self.ispaused = False
		self.ser.flushInput()			#serial buffer flushed on restart to prevent buildup
		while self.isrunning==True:
			self.ser.flushInput()
			while self.ispaused==False:	
				msg = self.ser.readline()
				try:
					msg2 = str(msg.decode('ascii'))
					if msg2[0:6] == '$GNGGA':
						if msg2.split(',')[6] == '1':
							lat, latdir, lon, londir, fix, sats = msg2.split(',')[2:8]
							if latdir == 'N':
								latdir = 1
							else:
								latdir = -1
								
							if londir == 'E':
								londir = 1
							else:
								londir = -1
							
							lat = latdir*(int(lat[:2])+(float(lat[2:])/60))
							lon = londir*(int(lon[:3])+(float(lon[3:])/60))
							east, north, znnum, znlet = utm.from_latlon(lat, lon, self.znnumdef, self.znletdef)
							east = round(east - self.Eoffset, 3)
							north = round(north - self.Noffset,3)
							self.Crosstrackerror(east, north)
							GPSlist = [fix, sats, east, north, znnum, znlet, self.XTE, self.lst]
							self.sig.emit(GPSlist)
						else:
							east, north, znnum, znlet, XTE, lst = None, None, None, '', 0, [0,0,0,0,0,0,0,0,0,0,0]
							fix, sats = msg2.split(',')[6:8]
							GPSlist = [fix, sats, east, north, znnum, znlet, self.XTE, self.lst]
							self.sig.emit(GPSlist)
				except:
					pass	


	def stop(self):
		print('stopping refresh')
		self.ispaused = True
		self.isrunning = False
		
	def Crosstrackerror(self, East=None, North=None):
		if not East==None and not North==None and not self.ABa==None and not self.ABb==None and not self.ABc==None:
			self.XTE = ((self.ABa * East) + (self.ABb * North) + self.ABc) / math.sqrt((self.ABa * self.ABa) + (self.ABb * self.ABb))
			self.lightbar(self.XTE)
			return
		else:
			self.XTE = 0
			return

#Don't recreate the list simply change the values, also dont return value just set the object variables			
	def lightbar(self, XTE):
		num = 5 + math.trunc(XTE / self.lightbarsensetivity)
		if num < 0:
			num = 0
		if num > 10:
			num = 10
		if num > 5:
			for i in range(11):
				if i<=num and i>=5:
					self.lst[i] = 1
				else:
					self.lst[i] = 0
					
		if num <= 5:
			for i in range(11):
				if i <=5 and i>=num:
					self.lst[i] = 1
				else:
					self.lst[i] = 0
		return
		
		
class HardwareOutputWorker(QObject):
	
	def __init__(self):
		super(HardwareOutputWorker, self).__init__()
		
	def setup(self):
		GPIO.setmode(GPIO.BCM)	
		#setup LEDs
		self.LEDs = [6,13,19,26,18,24,25,12,16,20,21]
		GPIO.setup(self.LEDs, GPIO.OUT)
		#self.lst = [0,0,0,0,0,0,0,0,0,0,0]
		
	def resetLED(self):
		#This is where the main while loop will be run looking for all continuous inputs i.e. buttons
		GPIO.output(self.LEDs, 0)
	
	def refreshLED(self, x):
		for i in range(len(self.LEDs)):
			GPIO.output(self.LEDs[i],x[i])
			
			
	def destroy(self):
		print('killed')
		GPIO.output(self.LEDs, 0)
		#GPIO.cleanup()
		
		
class HardwareInputWorker(QObject):
	
	def __init__(self):
		super(HardwareInputWorker, self).__init__()
		GPIO.setmode(GPIO.BCM)

		
	def whileloop(self):
		#This is where the main while loop will be run looking for all continuous inputs i.e. buttons
		print('ji')
	
class GPS_Main_Window(QMainWindow):
	
	def __init__(self):
		super(GPS_Main_Window, self).__init__()

		self.setGeometry(0, 36, 1820, 1044)
		self.setWindowTitle('PyAgri GPS')
		
		#print(self.frameGeometry().x())
		#print(self.frameGeometry().y())
		#print(self.frameGeometry().width())
		#print(self.frameGeometry().height())
		
		self.initUI()
		#self.maximize()

		#print(self.frameGeometry().x())
		#print(self.frameGeometry().y())
		#print(self.frameGeometry().width())
		#print(self.frameGeometry().height())		
			
	def initUI(self):
		self.baseset = False
		self.ABflag = 0
		self.waitingdot = 1
		#---------------------------------------------------------------------------------
		self.scene = QGraphicsScene(self)
		self.graphicview = QGraphicsView(self.scene, self)
		self.graphicview.setFrameShape(QFrame.NoFrame)
		self.graphicview.setGeometry(300,0,1300,300)
		self.graphicview.setSceneRect(0,0,1300,300)
		self.graphicview.setStyleSheet('background: transparent')
		#---------------------------------------------------------------------------------
		#NEED TO FIX QGRAPHICITEMGROUP TO ALLOW CHANGING THE COLOR OF ALL OBJECTS
		self.lightbarlist = []
		for i in range(11):
			if i>=4 and i<=6:
				self.lightbarlist.append(self.scene.addEllipse((i*100)+1,50,50,50,QPen(QColor('black'), 2),QBrush(QColor(0,255,0,51))))
			elif (i>=2 and i<=3) or (i<=8 and i>=7):
				self.lightbarlist.append(self.scene.addEllipse((i*100)+1,50,50,50,QPen(QColor('black'), 2),QBrush(QColor(255,255,0,51))))
			else:
				self.lightbarlist.append(self.scene.addEllipse((i*100)+1,50,50,50,QPen(QColor('black'), 2),QBrush(QColor(255,0,0,51))))
		#self.lightbargroup = QGraphicsItemGroup()
		#for i in self.lightbarlist:
		#	self.lightbargroup.addToGroup(i)	
		#self.scene.addItem(self.lightbargroup)
		
		self.l1  = QtWidgets.QLabel(self)
		self.l1.setText('')
		self.l1.move(1500,50)
		
		self.l2  = QtWidgets.QLabel(self)
		self.l2.setText('')
		self.l2.move(1500,100)
		
		self.l3  = QtWidgets.QLabel(self)
		self.l3.setText('')
		self.l3.move(1500,150)
		
		self.l4  = QtWidgets.QLabel(self)
		self.l4.setText('')
		self.l4.move(1500,250)
		
		self.l5  = QtWidgets.QLabel(self)
		self.l5.setText('')
		self.l5.move(1500,300)
	
		self.b1 = QtWidgets.QPushButton(self)
		self.b1.setText('Start')
		self.b1.clicked.connect(self.b1clicked)
		self.b1.setGeometry(0,0,100,100)
		
		self.b2 = QtWidgets.QPushButton(self)
		self.b2.setText('Stop')
		self.b2.clicked.connect(self.b2clicked)
		self.b2.setGeometry(0,100,100,100)
		
		self.b3 = QtWidgets.QPushButton(self)
		self.b3.setText('Set Base\nPoint')
		self.b3.clicked.connect(self.b3clicked)
		self.b3.setGeometry(0,200,100,100)
		self.b3.setStyleSheet('background:rgb(0,255,0)')
		
		self.b4 = QtWidgets.QPushButton(self)
		self.b4.setText('Set AB Line')
		self.b4.clicked.connect(self.b4clicked)
		self.b4.setGeometry(0,300,100,100)
		
		self.b5 = QtWidgets.QPushButton(self)
		self.b5.setText('Test')
		self.b5.clicked.connect(self.b5clicked)
		self.b5.setGeometry(0,400,100,100)
		
		self.b6 = QtWidgets.QPushButton(self)
		self.b6.setText('Test 2')
		self.b6.clicked.connect(self.b6clicked)
		self.b6.setGeometry(0,500,100,100)
		

	def b1clicked(self):
		if self.baseset and not self.NMEA.isrunning:
			self.thrd.start()
			self.b1.setStyleSheet('')
			self.b1.setText('Pause')
			
		elif self.baseset and self.NMEA.ispaused:
			self.NMEA.ispaused = False
			self.b1.setStyleSheet('')
			self.b1.setText('Pause')
			
		elif self.baseset and not self.NMEA.ispaused:
			self.NMEA.ispaused = True
			self.b1.setStyleSheet('background:rgb(0,255,0)')
			self.b1.setText('Start')
		
	def b2clicked(self):
		self.NMEA.isPaused = True
		self.NMEA.stop()
		self.thrd.quit()
		self.thrd.wait()
		
		self.b1.setStyleSheet('background:rgb(0,255,0)')
		self.b1.setText('Start')
		
		for i in range(11):
			if i>=4 and i<=6:
				self.lightbarlist[i].setBrush(QBrush(QColor(0, 255, 0, 51)))
			elif (i>=2 and i<=3) or (i<=8 and i>=7):
				self.lightbarlist[i].setBrush(QBrush(QColor(255, 255, 0, 51)))
			else:
				self.lightbarlist[i].setBrush(QBrush(QColor(255, 0, 0, 51)))
		print('kill')
		self.HardwareOutput.destroy()
		self.thrd2.quit()
		self.thrd2.wait()
		
	def b3clicked(self):
		if not self.baseset:
			self.fix, self.sats, self.east, self.north, self.znnum, self.znlet = self.setbase()
			#recheck if the base has been successfully set
			if self.baseset:
				self.l3.setText(str(round(self.east,3))+'E, ' + str(round(self.north,3))+'N\n'+str(self.znnum) + self.znlet)
				self.l3.adjustSize()
				self.Eoffset = round(self.east,0)
				self.Noffset = round(self.north,0)
				
				self.NMEA = NMEAworker(self.znnum, self.znlet, self.Eoffset, self.Noffset)
				self.thrd = QThread()
				self.NMEA.sig.connect(self.refresh)
				self.NMEA.moveToThread(self.thrd)	
				self.thrd.started.connect(self.NMEA.NMEAtoUTM)
				
				self.HardwareOutput = HardwareOutputWorker()
				self.thrd2 = QThread()
				self.HardwareOutput.moveToThread(self.thrd2)
				self.thrd2.start()
				self.thrd2.started.connect(self.HardwareOutput.setup)
				
				self.b3.setStyleSheet('')
				self.b1.setStyleSheet('background:rgb(0,255,0)')
				self.l3.setStyleSheet('')
				
			else:
				self.l3.setText('No GPS\nBase Not Set')
				self.l3.setStyleSheet('font: bold; color: rgb(255,0,0) ')
				self.l3.adjustSize()

	def b4clicked(self):
		try:
			if self.ABflag == 0:
				self.b4.setText('Set A')
				self.b4.setStyleSheet('background: rgb(0,255,255)')
				self.ABflag = 1
			
			elif self.ABflag == 1:
				self.Aeast = self.east
				self.Anorth = self.north
				self.b4.setText('Set B')
				self.b4.setStyleSheet('background:rgb(255,0,255)')
				self.ABflag = 2

			else:
				self.Beast = self.east
				self.Bnorth = self.north
				self.b4.setText('Set AB\nLine')
				self.b4.setStyleSheet('')
				self.ABflag = 0
				self.ABline()
		except:
			pass
			
	def b5clicked(self):
		print(time.time())
		for i in self.lightbarlist:
			i.setBrush(QBrush(QColor(0,255,0,255)))
		print(time.time())

	def b6clicked(self):
		print(time.time())
		self.setUpdatesEnabled(False)
		for i in self.lightbarlist:
			i.setBrush(QBrush(QColor(255,0,0,255)))
		self.setUpdatesEnabled(True)
		print(time.time())
			
	def refresh(self, sig):	
		print('refresh')
		self.fix =sig[0]
		self.sats =sig[1]
		self.east =sig[2]
		self.north =sig[3]
		self.znnum =sig[4]
		self.znlet =sig[5]
		self.XTE =sig[6]
		self.lst =sig[7]

		if self.fix == '1':
			self.fixed = 'GPS Fix'
			self.l1.setText(str(self.east)+'E, ' + str(self.north)+'N\n'+str(round(self.XTE,3))+'m')
			self.l1.adjustSize()

		else:
			self.fixed = 'No GPS Fix'
			self.l1.setText('.'*self.waitingdot)
			if self.waitingdot < 3:
				self.waitingdot += 1
			else:
				self.waitingdot = 1
		self.l2.setText(self.fixed + '    ' + self.sats + ' Satellites')
		self.l2.adjustSize()
		
		for i in range(11):
			if i>=4 and i<=6:
				self.lightbarlist[i].setBrush(QBrush(QColor(0 ,255 ,0 ,(51+(255*self.lst[i]*0.8)))))
			elif (i>=2 and i<=3) or (i<=8 and i>=7):
				self.lightbarlist[i].setBrush(QBrush(QColor(255 ,255 ,0 ,(51+(255*self.lst[i]*0.8)))))
			else:
				self.lightbarlist[i].setBrush(QBrush(QColor(255,0 ,0 ,(51+(255*self.lst[i]*0.8)))))
		self.HardwareOutput.refreshLED(self.lst)
		
	def setbase(self, znnumdef=None, znletdef=None):
		self.prt = '/dev/ttyACM0'
		self.bd = 115200
		self.ser = serial.Serial(self.prt, self.bd)
		self.ser.flushInput()
		while True:
			msg = self.ser.readline()
			try:
				msg2 = str(msg.decode('ascii'))
				if msg2[0:6] == '$GNGGA':
					if msg2.split(',')[6] == '1':
						lat, latdir, lon, londir, fix, sats = msg2.split(',')[2:8]
						if latdir == 'N':
							latdir = 1
						else:
							latdir = -1
							
						if londir == 'E':
							londir = 1
						else:
							londir = -1
						
						lat = latdir*(int(lat[:2])+(float(lat[2:])/60))
						lon = londir*(int(lon[:3])+(float(lon[3:])/60))
						east, north, znnum, znlet = utm.from_latlon(lat, lon, znnumdef, znletdef)
						GPSlist = [fix, sats, east, north, znnum, znlet]
						self.baseset = True
						return GPSlist
					
					else:
						east, north, znnum, znlet = None, None, None, ''
						fix, sats = msg2.split(',')[6:8]
						GPSlist = [fix, sats, east, north, znnum, znlet]
						return GPSlist
			except:
				pass		

	def ABline(self):
		self.dy = self.Bnorth - self.Anorth
		self.dx = self.Beast - self.Aeast
		#print([self.Aeast, self.Anorth, self.Beast, self.Bnorth])
		#print([self.dx, self.dy])
		#check if Point A and Point B are not the same i.e. a line
		if self.dy != 0 and self.dx != 0:
			#then need to check what the line is doing i.e. horizontal or vertical
			self.ABheading = math.atan2(self.dx, self.dy)
			#print(self.ABheading)
			self.ABa = self.dy
			self.ABb = - self.dx
			self.ABc = - (self.ABa * self.Aeast) - (self.ABb * self.Anorth)
			#print([self.ABa, self.ABb, self.ABc]) 
			#print(((self.ABheading*180/np.pi)+360) % 360)
			self.l4.setText(str(round((((self.ABheading*180/math.pi)+360) % 360), 1)) + '°')
			self.l4.adjustSize()
			self.NMEA.ABa = self.ABa
			self.NMEA.ABb = self.ABb
			self.NMEA.ABc = self.ABc
		

def window():
	app = QApplication(sys.argv)
	app.setStyle('Fusion')
	win = GPS_Main_Window()
	win.show()
	sys.exit(app.exec_())

if __name__ == '__main__':
	window()
