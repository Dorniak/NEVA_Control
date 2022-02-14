from PyQt5.QtWidgets import (
    QSlider, QStyleOptionSlider, 
    QStyle, QLabel, 
    QDialog, QGridLayout, 
    QSizePolicy
    )
from PyQt5.QtCore import QThread, pyqtSignal, Qt
import urllib.request
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import math
import numpy as np
import random
import time
import os
# import getlidarMain
# import LidarConfig1
# import filtBy
# import sqlite3

class StreamViewerThread(QThread):
    give_me_frame = pyqtSignal(object)
    stream_not_found = pyqtSignal(str)
    end = pyqtSignal(str)

    def __init__(self,parent):
        super().__init__()
        self.parent = parent
        self.running = True
        
    def run(self):
        try:
            self.running = True
            stream = urllib.request.urlopen('http://192.168.18.15:9090/?action=stream')
        except:
            msg = 'Connection refused please make sure your mjpg streamer server is running'
            self.stream_not_found.emit(msg)
            self.running = False
        bytes = b''
        while self.running:
            try:
                res = urllib.request.urlopen('http://192.168.18.15:9090/?action=stream')
            except:
                msg = 'Connection refused please make sure your mjpg streamer server is running'
                self.stream_not_found.emit(msg)
                self.running = False
                break
            bytes += stream.read(1024)
            a = bytes.find(b'\xff\xd8')
            b = bytes.find(b'\xff\xd9')
            if a != -1 and b != -1:
                jpg = bytes[a:b+2]
                bytes = bytes[b+2:]
                self.give_me_frame.emit(jpg)

    def stop(self):
        self.running = False


class MplCanvas(FigureCanvas):

    def __init__(self, parent=None, dpi=100, width=None, height=None):
        self.fig = Figure(figsize=(width, height), dpi=dpi) #,facecolor='black'
        self.axes = self.fig.add_subplot(111)#, projection='3d') #TODO: No funciona usar projection 3d
        super(MplCanvas, self).__init__(self.fig)

class GraphThread(QThread):
    end = pyqtSignal(str)

    def __init__(self,parent):
        super().__init__()
        self.parent = parent
        self.running = True
        n_data = 100
        self.xdata = list(range(n_data))
        self.ydata = [random.randint(0, 10) for i in range(n_data)]
        self.zdata = self.ydata
        self.cdata = 'r'
        #print('Llego hasta este punto(5)')
        #print(self.parent.parent)
        #print(self.parent.parent.AcquisitionModule.res[0,1:])
        #print(self.parent.parent.res[0,1:])

    def run(self):
        while self.running:
            self.update_plot()
            time.sleep(0.4)

    def update_plot(self):
        # Drop off the first y element, append a new one.
        self.ydata = self.ydata[1:] + [random.randint(0, 10)]
        #self.xdata = self.parent.parent.AcquisitionModule.res[0,1:]
        #self.ydata = self.parent.parent.AcquisitionModule.res[1,1:]
        #self.zdata = self.parent.parent.AcquisitionModule.res[2,1:]
        #self.cdata = self.parent.parent.AcquisitionModule.res[3,1:]
        self.parent.canvas.axes.cla()  # Clear the canvas.
        self.parent.canvas.axes.plot(self.xdata, self.ydata, self.zdata, self.cdata)
        # Trigger the canvas to update and redraw.
        self.parent.canvas.draw()

    def stop(self):
        self.running = False

""" class AcquisitionThread(QThread):
    end = pyqtSignal(str)
    def __init__(self, parent):
        super().__init__()
        self.parent = parent
        self.running = True
        #Devuelve el incremento min. de azimut dependiendo de la velocidad de rot. del LiDAR
        #Necesitamos visibilidad de estos valores dentro del archivo principal. 
        self.lidarconf1 = self.parent.lidarconf1
        self.altMat1 = self.lidarconf1.getAltMat1()
        self.t_offset =  self.lidarconf1.get_t_offset()
        self.lidarSpeed = self.lidarconf1.getLidarSpeed()
        self.sock = self.lidarconf1.getMsopSocket()
        self.deltaA = getlidarMain.getDeltaAz(self.lidarSpeed)
        self.msize = math.floor(360/self.deltaA)
        self.M = np.zeros((17,self.msize,2))
        self.R = np.zeros((17,self.msize,3)) 
        self.resultadoR = np.zeros((2,3))
        self.resultadoM = np.zeros((2,3))
        self.res = np.zeros((2,3))
        self.dis = np.zeros((2,3))

        #Como consecuencia de todos los pasos anteriores, podemos definir ma matriz de distancias y reflectividades M y la matrix de posiciones en cartesianas R
        #M[0,:,0] y M[0,:,1] contienen los azimuts
        #El M[1:,:,0] corresponde a las distancias y M[1:,:,1] corresponde a las reflectividades
        #R[0,:,0] y R[0,:,1] contienen los azimuts
        #R[1:,:,0] corresponde a las distancias en el eje X, R[1:,:,1] a las distancias en Y, y R[1:,:,2] corresponde a las distancias en Z
        # Hay que definirlas en este momento porque si no, hemos de crearlas dentro de la funci. de lectura lo cual es muy ineficiente.  
        self.conn = self.parent.conn
        self.cur  = self.parent.cur
        
        
        #resultadoR, resultadoM, res, dis = filtBy.filtByAngle(lidConf = self.lidarconf1, M=M, R=R, conn=self.conn, azmlmin=0., azmlmax=360., altmin=-16., altmax=16., pub="n") #s
        #self.xdata = res[0,:]
        #self.ydata = res[1,:]
        #self.zdata = res[2,:]
        #self.rdata = res[3,:]
        #print(res)
    def run(self):
        while self.running:
            self.update_data()
            time.sleep(0.01)

    def update_data(self):
        # Drop off the first y element, append a new one.
        self.R, self.M = getlidarMain.getAll(lidConf = self.lidarconf1, M=self.M, R=self.R, conn=self.conn, pub="n") #s
        self.resultadoR, self.resultadoM, self.res, self.dis = filtBy.filtByAngle(lidConf = self.lidarconf1, M=self.M, R=self.R, conn=self.conn, azmlmin=0., azmlmax=360., altmin=-16., altmax=16., pub="n") #s
        
        #self.xdata = res[0,1:]
        #self.ydata = res[1,1:]
        #self.zdata = res[2,1:]
        #self.rdata = res[3,1:]
        
        #print(self.res,'\n',self.dis)
        return(self.resultadoR, self.resultadoM, self.res, self.dis)

    def stop(self):
        self.running = False
 """
 
class Slider(QSlider):
    val = 0
    def mousePressEvent(self, event):
        super(Slider, self).mousePressEvent(event)
        if event.button() == Qt.LeftButton:
            self.val = self.pixelPosToRangeValue(event.pos())
            self.setValue(self.val)

    def pixelPosToRangeValue(self, pos):
        opt = QStyleOptionSlider()
        self.initStyleOption(opt)
        gr = self.style().subControlRect(QStyle.CC_Slider, opt, QStyle.SC_SliderGroove, self)
        sr = self.style().subControlRect(QStyle.CC_Slider, opt, QStyle.SC_SliderHandle, self)

        if self.orientation() == Qt.Horizontal:
            sliderLength = sr.width()
            sliderMin = gr.x()
            sliderMax = gr.right() - sliderLength + 1
        else:
            sliderLength = sr.height()
            sliderMin = gr.y()
            sliderMax = gr.bottom() - sliderLength + 1
        pr = pos - sr.center() + sr.topLeft()
        p = pr.x() if self.orientation() == Qt.Horizontal else pr.y()
        return QStyle.sliderValueFromPosition(self.minimum(), self.maximum(), p - sliderMin,
                                               sliderMax - sliderMin, opt.upsideDown)


class MessageDialog(QDialog):
    def __init__(self,parent,msg):
        super().__init__(parent)
        self.parent=parent
        self.msg = msg
        self.resize(600, 200)
        self.setStyleSheet('background-color: rgb(41, 36, 36)')
        self.SetupUI()
        self.setWindowModality(Qt.ApplicationModal)
        self.setWindowFlags(self.windowFlags() | Qt.CustomizeWindowHint)
        self.setWindowFlags(self.windowFlags() & ~Qt.WindowContextHelpButtonHint)

    def SetupUI(self):
        self.vpdbackground = QLabel(self)

        self.layout = QGridLayout(self)

        self.text = QLabel(self.msg,self)
        self.text.setStyleSheet("font-size:16px;color:white")

        self.layout.addWidget(self.vpdbackground,0,0,20,20)
        self.layout.addWidget(self.text,6,5,2,10)
        self.setLayout(self.layout)



class GooglePingThread(QThread):
    end = pyqtSignal(str)

    def __init__(self,parent):
        super().__init__()

        self.parent = parent

        self.running = True
        self.hostname = "google.com" #example
        self.count = 0

        
    def run(self):
        while self.running:
            response = os.system("ping -c 1 " + self.hostname)
            if response == 0:
                status = 'google is up'
                self.count +=1
            else:
                status = 'google is down'
            
            # for test 
            if self.count > 2:
                self.running = False
                status = 'google is down'
            self.end.emit(status)
            time.sleep(1)

    def stop(self):
        self.running = False


class YahooPingThread(QThread):
    end = pyqtSignal(str)

    def __init__(self,parent):
        super().__init__()

        self.parent = parent

        self.running = True
        self.hostname = "yahoo.com" #example
        self.count = 0

        
    def run(self):
        while self.running:
            response = os.system("ping -c 1 " + self.hostname)
            if response == 0:
                status = 'yahoo is up'
                self.count +=1
            else:
                status = 'yahoo is down'
            
            # for test 
            if self.count > 2:
                self.running = False
                status = 'yahoo is down'
            self.end.emit(status)
            time.sleep(1)

    def stop(self):
        self.running = False


class GmailPingThread(QThread):
    end = pyqtSignal(str)

    def __init__(self,parent):
        super().__init__()

        self.parent = parent

        self.running = True
        self.hostname = "gmail.com" #example
        self.count = 0

        
    def run(self):
        while self.running:
            response = os.system("ping -c 1 " + self.hostname)
            if response == 0:
                status = 'gmail is up'
                self.count +=1
            else:
                status = 'gmail is down'
            
            # for test 
            if self.count > 2:
                self.running = False
                status = 'gmail is down'
            self.end.emit(status)
            time.sleep(1)

    def stop(self):
        self.running = False

class youtubePingThread(QThread):
    end = pyqtSignal(str)

    def __init__(self,parent):
        super().__init__()

        self.parent = parent

        self.running = True
        self.hostname = "youtube.com" #example
        self.count = 0

        
    def run(self):
        while self.running:
            response = os.system("ping -n 1 " + self.hostname)
            if response == 0:
                status = 'youtube is up'
                self.count +=1
            else:
                status = 'youtube is down'
            
            # for test 
            if self.count > 2:
                self.running = False
                status = 'youtube is down'
            self.end.emit(status)
            time.sleep(1)

    def stop(self):
        self.running = False

class FacebookPingThread(QThread):
    end = pyqtSignal(str)

    def __init__(self,parent):
        super().__init__()

        self.parent = parent

        self.running = True
        self.hostname = "facebook.com" #example
        self.count = 0

        
    def run(self):
        while self.running:
            response = os.system("ping -c 1 " + self.hostname)
            if response == 0:
                status = 'facebook is up'
                self.count +=1
            else:
                status = 'facebook is down'
            
            # for test 
            if self.count > 2:
                self.running = False
                status = 'facebook is down'
            self.end.emit(status)
            time.sleep(1)

    def stop(self):
        self.running = False