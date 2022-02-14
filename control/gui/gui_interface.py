from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QWidget, QGridLayout, 
    QLabel, QApplication, 
    QCheckBox, QShortcut,
    QSizePolicy,
    )
from PyQt5 import (QtCore, QtGui, 
    QtWebEngineWidgets, QtWebChannel
    )
from matplotlib.mlab import window_none

from systemVerif import (
    SystemVerifRadioBtnWindow, 
    SystemNevaWindow, 
    ShowNevaImageWidget, 
    SystemVerifDriveCheckboxWindow,
    SystemVerifSpeedLimitCheckboxWindow,
    SystemVerifValuesWindow,
    ShowNevaLogoWidget
    )
from PyQt5.QtCore import QThread, pyqtSignal, Qt

""" from utils import (
    StreamViewerThread, MessageDialog, 
    MplCanvas, GraphThread, AcquisitionThread
    )
 """
from utils import (
    StreamViewerThread, MessageDialog, 
    MplCanvas, GraphThread
    )


import sys
import cv2
import numpy as np
import os
import math
import matplotlib
matplotlib.use('Qt5Agg')
import time
import LidarConfig1
import getlidarMain
import filtBy
import sqlite3

""" class AcquisitionModule():
    def __init__(self,parent):
        super().__init__()
        self.parent = parent
        UDP_IP = "192.168.0.102"
        UDP_PORT = 6699
        UDP_PORT_DIFOP = 7788
        self.lidarconf1 = LidarConfig1.LidarConfig1(UDP_IP=UDP_IP, UDP_PORT=UDP_PORT, UDP_PORT_DIFOP=UDP_PORT_DIFOP,graphadj='Full',secdis=1.0)
        self.conn = sqlite3.connect('file:cachedb?mode=memory&cache=shared')
        self.cur  = self.conn.cursor()
        self.AcquisitionThread = AcquisitionThread(self)
        #self.AcquisitionThread = AcquisitionThread(self, self.M,self.R, self.lidarconf1, self.conn)
        #self.checkbox_lidar_map.stateChanged.connect(self.parent.AcquisitionModule.getData)
        self.resultadoR = np.zeros((2,3))
        self.resultadoM = np.zeros((2,3))
        self.res = np.zeros((2,3))
        self.dis = np.zeros((2,3))

        self.getData()

        
    def getData(self):    
        self.AcquisitionThread.running = True
        self.AcquisitionThread.start()
        #self.resultadoR, self.resultadoM, self.res, self.dis = self.AcquisitionThread.update_data()
        self.R = self.AcquisitionThread.R
        self.M = self.AcquisitionThread.M
        self.resultadoR, self.resultadoM, self.res, self.dis = filtBy.filtByAngle(lidConf = self.lidarconf1, M=self.M, R=self.R, conn=self.conn, azmlmin=0., azmlmax=360., altmin=-16., altmax=16., pub="n") #s
        print(self.R,'\n', self.M,'\n')
        #print(self.resultadoR, '\n', self.resultadoM, '\n', self.res, '\n', self.dis)
        #self.resultadoM = self.AcquisitionThread.resultadoM
        #self.res = self.AcquisitionThread.res 
        #self.dis = self.AcquisitionThread.dis
        
        #self.parent.MatplotWindow.GraphThread.xdata = self.res[0,1:]
        #self.parent.MatplotWindow.GraphThread.ydata = self.res[1,1:]
        #self.parent.MatplotWindow.GraphThread.zdata = self.res[2,1:]
        #self.parent.MatplotWindow.GraphThread.cdata = self.res[3,1:]
        #print(self.M,self.R)
        #if not self.parent.MatplotWindow.checkbox_lidar_on.isChecked():
        #    self.AcquisitionThread.running = False
        #    self.AcquisitionThread.stop()
            
        #   self.set_tight_layout(True)
        # if self.parent.MatplotWindow.checkbox_lidar_map.isChecked():
        #     self.AcquisitionThread.running = True
        #     self.AcquisitionThread.start()
        #     self.R = self.AcquisitionThread.R
        #     self.M = self.AcquisitionThread.M
        #     print(self.M,self.R)
        #     #self.set_tight_layout(True)
        # else:
        #     self.AcquisitionThread.running = False
        #     self.AcquisitionThread.stop()

 """
class SystemWindow(QWidget):
    def __init__(self,parent):
        super().__init__()
        self.setStyleSheet("background-color: rgb(255, 255, 255)")
        self.SetupUI()

    def SetupUI(self):
        self.background = QLabel(self)
        self.layout = QGridLayout(self)

        self.SystemVerifRadioBtnWindow              = SystemVerifRadioBtnWindow(self)
        self.SystemVerifDriveCheckboxWindow         = SystemVerifDriveCheckboxWindow(self)

        self.ShowNevaLogoWidget                    = ShowNevaLogoWidget(self)
        self.ShowNevaLogoWidget.setStyleSheet("border: 1px solid black; border-left: 1px solid white;")

        self.SystemVerifSpeedLimitCheckboxWindow    = SystemVerifSpeedLimitCheckboxWindow(self)
        self.SystemVerifValuesWindow                = SystemVerifValuesWindow(self)

        self.ShowNevaImageWidget                    = ShowNevaImageWidget(self)
        self.SystemNevaWindow                       = SystemNevaWindow(self,self.SystemVerifSpeedLimitCheckboxWindow,self.ShowNevaImageWidget)
        
        self.layout.addWidget(self.background,1,1,20,20)

        self.layout.addWidget(self.SystemVerifRadioBtnWindow,1,1,15,5)
        self.layout.addWidget(self.SystemVerifDriveCheckboxWindow,1,6,10,5)
        self.layout.addWidget(self.SystemVerifSpeedLimitCheckboxWindow,1,11,10,6)
        self.layout.addWidget(self.ShowNevaLogoWidget,1,16,10,5)
        self.layout.addWidget(self.SystemVerifValuesWindow,11,6,5,15)

        self.layout.addWidget(self.SystemNevaWindow,16,1,5,20)
        self.layout.addWidget(self.ShowNevaImageWidget,18,19)

class MatplotWindow(QWidget):
    def __init__(self,parent):
        super().__init__()
        self.parent = parent
        self.setStyleSheet("background-color: rgb(255, 255, 255)")
        self.double_click_status = False
        self.SetupUI()

    def SetupUI(self):
        self.background = QLabel(self)
        self.background.setStyleSheet("background-color: rgb(255, 255, 255)")
        self.layout = QGridLayout(self)

        self.showGraphlabel = QLabel(self)
        self.showGraphlabel.setStyleSheet("border: 1px solid black;")

        self.checkbox_lidar_on = QCheckBox('Lidar on')
        self.checkbox_lidar_on.setChecked(True)
        self.checkbox_lidar_on.setStyleSheet("QCheckBox"
                                        "{spacing : 20px; color: black; font-size:18px; font-family: Arial};"
                                        "QCheckBox::indicator { width: 25px; height: 25px; spacing : 20px};")
        self.checkbox_lidar_on.stateChanged.connect(self.on_change_linder_checkbox)

        self.checkbox_lidar_graph = QCheckBox('Lidar graph')
        self.checkbox_lidar_graph.setStyleSheet("QCheckBox"
                                        "{spacing : 20px; color: black; font-size:18px; font-family: Arial};"
                                        "QCheckBox::indicator { width: 25px; height: 25px; spacing : 20px};")
        self.checkbox_lidar_graph.stateChanged.connect(self.set_graph)

        self.GraphThread = GraphThread(self)

        self.checkbox_lidar_map = QCheckBox('Lidar mapping')
        self.checkbox_lidar_map.setStyleSheet("QCheckBox"
                                        "{spacing : 20px; color: black; font-size:18px; font-family: Arial};"
                                        "QCheckBox::indicator { width: 25px; height: 25px; spacing : 20px};")
        self.canvas = MplCanvas(self, width=1.0, height=1.0, dpi=100)
        self.canvas.hide()

        self.shortcut_key_esc = QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Escape), self)
        self.shortcut_key_esc.activated.connect(self.on_esc_btn)

        self.layout.addWidget(self.background,1,1,20,20)
        self.layout.addWidget(self.showGraphlabel,2,2,16,18)
        self.layout.addWidget(self.checkbox_lidar_on,18,4,2,2)
        self.layout.addWidget(self.checkbox_lidar_graph,18,10,2,2)
        self.layout.addWidget(self.checkbox_lidar_map,18,16,2,2)
        self.layout.addWidget(self.canvas,2,2,16,18)

    def on_change_linder_checkbox(self):
        if self.checkbox_lidar_on.isChecked() == False:
            self.checkbox_lidar_graph.setChecked(False)

    def set_graph(self):
        if self.checkbox_lidar_on.isChecked() and self.checkbox_lidar_graph.isChecked():
            self.GraphThread.running = True
            self.GraphThread.start()
            self.canvas.show()
        else:
            self.GraphThread.running = False
            self.GraphThread.stop()
            self.canvas.close()

    
    def mouseDoubleClickEvent(self, e):
        self.current_window = self.parent.current_windows
        if self.double_click_status == False:
            self.double_click_status= True
            self.current_window[0].hide()
            self.current_window[2].hide()
            self.current_window[3].hide()
            self.background.hide()
            self.setStyleSheet("background-color: rgb(133, 133, 133)")
            self.parent.layout.addWidget(self,0,0,20,20)
        else:
            self.on_esc_btn()

    def on_esc_btn(self):
        self.GraphThread.running = False
        self.GraphThread.stop()
        self.current_window = self.parent.current_windows
        if self.double_click_status == True:
            self.double_click_status = False
            self.setStyleSheet("background-color: rgb(255, 255, 255)")
            self.current_window[0].show()
            self.current_window[2].show()
            self.current_window[3].show()
            self.background.show()

            self.parent.layout.addWidget(self,1,10,9,9)
            
            self.canvas.close()
            self.canvas.deleteLater()
            time.sleep(0.3)
            self.canvas = MplCanvas(self, width=1.0, height=1.0, dpi=100)
            self.canvas.hide()
            self.layout.addWidget(self.canvas,2,2,16,18)
            if self.checkbox_lidar_graph.isChecked() and self.checkbox_lidar_on.isChecked():
                print('graph and linder is checked')
                self.GraphThread.running = True
                self.GraphThread.start()
                self.canvas.show()
            if self.parent.isMaximized() or self.parent.isFullScreen():
                pass
            else:
                self.parent.showMaximized()
                

class CameraWindow(QWidget):
    def __init__(self,parent):
        super().__init__()
        self.setStyleSheet("background-color: rgb(255, 255, 255)")
        self.double_click_status = False
        self.parent = parent
        self.SetupUI()

    def SetupUI(self):
        self.background = QLabel(self)
        self.background.setStyleSheet("background-color: rgb(255, 255, 255)")
        self.layout = QGridLayout(self)

        self.videodisplaylabel = QLabel(self)
        self.videodisplaylabel.setStyleSheet("border: 1px solid black;background-color: rgb(0, 0, 0)")
        self.videodisplaylabel.adjustSize()

        self.checkbox_camera_on = QCheckBox('Camera on')
        self.checkbox_camera_on.setStyleSheet("QCheckBox"
                                        "{spacing : 20px; color: black; font-size:18px; font-family: Arial};"
                                        "QCheckBox::indicator { width: 25px; height: 25px; spacing : 20px};")
        self.checkbox_camera_on.setChecked(True)
        self.checkbox_camera_on.stateChanged.connect(self.on_camera_checked)

        self.checkbox_camera_record = QCheckBox('Camera record')
        self.checkbox_camera_record.setStyleSheet("QCheckBox"
                                        "{spacing : 20px; color: black; font-size:18px; font-family: Arial};"
                                        "QCheckBox::indicator { width: 25px; height: 25px; spacing : 20px};")

        
        self.checkbox_camera_graph = QCheckBox('Camera graph')
        self.checkbox_camera_graph.setStyleSheet("QCheckBox"
                                        "{spacing : 20px; color: black; font-size:18px; font-family: Arial};"
                                        "QCheckBox::indicator { width: 25px; height: 25px; spacing : 20px};")



        self.Volante_label = QLabel('Volante',self)
        self.Volante_label.setFont(QtGui.QFont('Arial', 11))
        self.Volante_label.setAlignment(Qt.AlignCenter)
        self.Volante_label.setStyleSheet('background-color: black; color: white;')
        self.Volante_label.adjustSize()

        self.Volante_no_label = QLabel('0',self)
        self.Volante_no_label.setFont(QtGui.QFont('Arial', 11))
        self.Volante_no_label.setAlignment(Qt.AlignCenter)
        self.Volante_no_label.setStyleSheet('background-color: black; color: white;')
        self.Volante_no_label.adjustSize()

        self.Velocidad_label = QLabel('Velocidad',self)
        self.Velocidad_label.setFont(QtGui.QFont('Arial', 11))
        self.Velocidad_label.setAlignment(Qt.AlignCenter)
        self.Velocidad_label.setStyleSheet('background-color: black; color: white;')
        self.Velocidad_label.adjustSize()

        self.Velocidad_label_no = QLabel('0',self)
        self.Velocidad_label_no.setFont(QtGui.QFont('Arial', 11))
        self.Velocidad_label_no.setAlignment(Qt.AlignCenter)
        self.Velocidad_label_no.setStyleSheet('background-color: black; color: white;')
        self.Velocidad_label_no.adjustSize()

        self.freno_label = QLabel('Freno',self)
        self.freno_label.setFont(QtGui.QFont('Arial', 11))
        self.freno_label.setAlignment(Qt.AlignCenter)
        self.freno_label.setStyleSheet('background-color: black; color: white;')
        self.freno_label.adjustSize()

        self.freno_label_no = QLabel('0',self)
        self.freno_label_no.setFont(QtGui.QFont('Arial', 11))
        self.freno_label_no.setAlignment(Qt.AlignCenter)
        self.freno_label_no.setStyleSheet('background-color: black; color: white;')
        self.freno_label_no.adjustSize()

        self.p_emergencia_label = QLabel('P Emergencia',self)
        self.p_emergencia_label.setFont(QtGui.QFont('Arial', 11))
        self.p_emergencia_label.setAlignment(Qt.AlignCenter)
        self.p_emergencia_label.setStyleSheet('background-color: black; color: white;')
        self.p_emergencia_label.adjustSize()

        self.p_emergencia_label_no = QLabel('False',self)
        self.p_emergencia_label_no.setFont(QtGui.QFont('Arial', 11))
        self.p_emergencia_label_no.setAlignment(Qt.AlignCenter)
        self.p_emergencia_label_no.setStyleSheet('background-color: black; color: white;')
        self.p_emergencia_label_no.adjustSize()

        self.marcha_label = QLabel('Marcha',self)
        self.marcha_label.setFont(QtGui.QFont('Arial', 11))
        self.marcha_label.setAlignment(Qt.AlignCenter)
        self.marcha_label.setStyleSheet('background-color: black; color: white;')
        self.marcha_label.adjustSize()

        self.marcha_label_no = QLabel('?',self)
        self.marcha_label_no.setFont(QtGui.QFont('Arial', 11))
        self.marcha_label_no.setAlignment(Qt.AlignCenter)
        self.marcha_label_no.setStyleSheet('background-color: black; color: white;')
        self.marcha_label_no.adjustSize()


        self.thread = StreamViewerThread(self)
        self.thread.give_me_frame.connect(self.on_show_stream)
        self.thread.stream_not_found.connect(self.on_mjpg_connectiion_failed)
        self.thread.start()

        self.shortcut_key_esc = QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Escape), self)
        self.shortcut_key_esc.activated.connect(self.on_esc_btn)

        self.layout.addWidget(self.background,1,1,20,20)
        self.layout.addWidget(self.videodisplaylabel,2,2,16,18)

        self.layout.addWidget(self.Volante_label,4,3)
        self.layout.addWidget(self.Volante_no_label,5,3)
        self.layout.addWidget(self.Velocidad_label,8,3)
        self.layout.addWidget(self.Velocidad_label_no,9,3)
        self.layout.addWidget(self.freno_label,12,3)
        self.layout.addWidget(self.freno_label_no,13,3)
        self.layout.addWidget(self.p_emergencia_label,4,18)
        self.layout.addWidget(self.p_emergencia_label_no,5,18)
        self.layout.addWidget(self.marcha_label,8,18)
        self.layout.addWidget(self.marcha_label_no,9,18)

        self.layout.addWidget(self.checkbox_camera_on,18,4,2,2)
        self.layout.addWidget(self.checkbox_camera_graph,18,10,2,2)
        self.layout.addWidget(self.checkbox_camera_record,18,16,2,2)
        self.videodisplaylabel_height = None
        self.videodisplaylabel_width = None

    def on_camera_checked(self):
        if self.checkbox_camera_on.isChecked():
            self.thread.is_running = True
            self.thread.start()
        else:
            self.thread.stop()
            canvas = QtGui.QPixmap(150, 100)
            canvas.fill(QtGui.QColor("black"))
            self.videodisplaylabel.setPixmap(canvas)

    def on_show_stream(self,image):
        if self.thread.running:
            image = cv2.imdecode(np.fromstring(image, dtype=np.uint8), cv2.IMREAD_COLOR)
            height, width, channel = image.shape
            step = channel * width
            qImg = QtGui.QImage(image.data, width, height, step, QtGui.QImage.Format_BGR888)
            
            if self.double_click_status:
                if self.videodisplaylabel_height is None:
                    self.videodisplaylabel_height = self.videodisplaylabel.height()
                    self.videodisplaylabel_width = self.videodisplaylabel.width()
                if self.parent.isMaximized() or self.parent.isFullScreen():
                    q = qImg.scaled(height  ,width, QtCore.Qt.KeepAspectRatioByExpanding)
                    self.videodisplaylabel.setAlignment(QtCore.Qt.AlignmentFlag.AlignHCenter| QtCore.Qt.AlignmentFlag.AlignVCenter)
                    self.videodisplaylabel.setPixmap(QtGui.QPixmap.fromImage(q))
                else:
                    q = qImg.scaled(height  ,width)
                    self.videodisplaylabel.setAlignment(QtCore.Qt.AlignmentFlag.AlignHCenter| QtCore.Qt.AlignmentFlag.AlignVCenter)
                    self.videodisplaylabel.setPixmap(QtGui.QPixmap.fromImage(q))
            else:
                if self.videodisplaylabel_height is None:
                    self.videodisplaylabel_height = self.videodisplaylabel.height()
                    self.videodisplaylabel_width = self.videodisplaylabel.width()
                self.our_height = height - self.videodisplaylabel_height - 100
                if self.parent.isMaximized() or self.parent.isFullScreen():
                    q = qImg.scaled(self.videodisplaylabel_height + self.our_height, width, QtCore.Qt.KeepAspectRatio)
                    self.videodisplaylabel.setAlignment(QtCore.Qt.AlignmentFlag.AlignHCenter| QtCore.Qt.AlignmentFlag.AlignVCenter)
                    self.videodisplaylabel.setPixmap(QtGui.QPixmap.fromImage(q))
                else:
                    q = qImg.scaled(self.videodisplaylabel_height + self.our_height, width -500, QtCore.Qt.KeepAspectRatio)
                    self.videodisplaylabel.setAlignment(QtCore.Qt.AlignmentFlag.AlignHCenter| QtCore.Qt.AlignmentFlag.AlignVCenter)
                    self.videodisplaylabel.setPixmap(QtGui.QPixmap.fromImage(q))

    def on_mjpg_connectiion_failed(self,msg):
        self.checkbox_camera_on.setChecked(False)
        canvas = QtGui.QPixmap(150, 100)
        canvas.fill(QtGui.QColor("black"))
        self.videodisplaylabel.setPixmap(canvas)
        self.MessageDialog = MessageDialog(self,msg)
        self.MessageDialog.show()

    def mouseDoubleClickEvent(self, e):
        self.current_window = self.parent.current_windows
        if self.double_click_status == False:
            self.double_click_status= True
            self.current_window[0].hide()
            self.current_window[1].hide()
            self.current_window[3].hide()
            self.background.hide()
            self.setStyleSheet("background-color: rgb(133, 133, 133)")
            self.parent.layout.addWidget(self,0,0,20,20)
        else:
            self.on_esc_btn()

    def on_esc_btn(self):
        self.current_window = self.parent.current_windows
        if self.double_click_status == True:
            self.double_click_status = False
            self.background.show()
            self.setStyleSheet("background-color: rgb(255, 255, 255)")
            self.current_window[0].show()
            self.current_window[1].show()
            self.current_window[3].show()
            if self.parent.isMaximized() or self.parent.isFullScreen():
                pass
            else:
                self.parent.showMaximized()
            self.parent.layout.addWidget(self,10,1,9,9)


class MapWindow(QWidget):
    def __init__(self,parent):
        super().__init__()
        self.setStyleSheet("background-color: rgb(255, 255, 255)")
        self.parent = parent
        self.double_click_status = False
        self.SetupUI()

    def SetupUI(self):
        self.background = QLabel(self)
        self.background.setStyleSheet("background-color: rgb(255, 255, 255)")
        self.layout = QGridLayout(self)

        self.mapdisplaylabel = QLabel(self)
        self.mapdisplaylabel.setStyleSheet("border: 1px solid black;")

        self.checkbox_gps_on = QCheckBox('GPS on')
        self.checkbox_gps_on.setStyleSheet("QCheckBox"
                                        "{spacing : 20px; color: black; font-size:18px; font-family: Arial};"
                                        "QCheckBox::indicator { width: 25px; height: 25px; spacing : 20px};")
        self.checkbox_gps_on.stateChanged.connect(self.set_map)

        label = self.label = QLabel()
        sp = QSizePolicy()
        sp.setVerticalStretch(0)
        label.setSizePolicy(sp)
        view = self.view = QtWebEngineWidgets.QWebEngineView()
        channel = self.channel = QtWebChannel.QWebChannel()

        channel.registerObject("MapWindow", self)
        view.page().setWebChannel(channel)

        file = os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            "src/map.html",
        )
        self.view.setUrl(QtCore.QUrl.fromLocalFile(file))
        self.view.hide()

        self.shortcut_key_esc = QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Escape), self)
        self.shortcut_key_esc.activated.connect(self.on_esc_btn)

        self.layout.addWidget(self.background,1,1,20,20)
        self.layout.addWidget(self.mapdisplaylabel,2,2,16,18)
        self.layout.addWidget(self.view,2,2,16,18)
        self.layout.addWidget(self.checkbox_gps_on,18,4,2,2)

    def set_map(self):
        if self.checkbox_gps_on.isChecked():
            self.view.close()
            self.mapdisplaylabel.close()
            self.view.show()
        else:
            self.view.close()
            self.mapdisplaylabel.show()
    
    @QtCore.pyqtSlot(float, float)
    def onMapMove(self, lat, lng):
        self.label.setText("Lng: {:.5f}, Lat: {:.5f}".format(lng, lat))

    def panMap(self, lng, lat):
        page = self.view.page()
        page.runJavaScript("map.panTo(L.latLng({}, {}));".format(lat, lng))
    
    def mouseDoubleClickEvent(self, e):
        self.current_window = self.parent.current_windows
        if self.double_click_status == False:
            self.double_click_status= True
            self.current_window[0].hide()
            self.current_window[1].hide()
            self.current_window[2].hide()
            self.background.hide()
            self.setStyleSheet("background-color: rgb(133, 133, 133)")
            self.parent.layout.addWidget(self,0,0,20,20)
        else:
            self.on_esc_btn()

    def on_esc_btn(self):
        self.current_window = self.parent.current_windows
        if self.double_click_status == True:
            self.double_click_status = False

            self.background.show()
            self.setStyleSheet("background-color: rgb(255, 255, 255)")
            self.current_window[0].show()
            self.current_window[1].show()
            self.current_window[2].show()
            self.parent.layout.addWidget(self,10,10,9,9)
            if self.checkbox_gps_on.isChecked():
                self.checkbox_gps_on.setChecked(False)
                self.checkbox_gps_on.setChecked(True)

            if self.parent.isMaximized() or self.parent.isFullScreen():
                pass
            else:
                self.parent.showMaximized()



class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.current_windows = []
        self.layout = QGridLayout(self)
        self.setWindowTitle('Neva')
        self.setWindowFlag(Qt.WindowMinimizeButtonHint, True)
        self.setWindowFlag(Qt.WindowMaximizeButtonHint, True)

        self.setStyleSheet("background-color: rgb(133, 133, 133)")

        self.SystemWindow   = SystemWindow(self)
        self.MatplotWindow  = MatplotWindow(self)
        self.CameraWindow   = CameraWindow(self)
        self.MapWindow      = MapWindow(self)
        #self.AcquisitionModule = AcquisitionModule(self)
        
        #self.AcquisitionModule = AcquisitionModule(self)
        #self.AcquisitionModule.AcquisitionThread.res
        



        self.layout.addWidget(self.SystemWindow,1,1,9,9)
        self.layout.addWidget(self.MatplotWindow,1,10,9,9)
        self.layout.addWidget(self.CameraWindow,10,1,9,9)
        self.layout.addWidget(self.MapWindow,10,10,9,9)

        self.current_windows.append(self.SystemWindow)
        self.current_windows.append(self.MatplotWindow)
        self.current_windows.append(self.CameraWindow)
        self.current_windows.append(self.MapWindow)
        self.setLayout(self.layout)

class prueba(QtCore.QObject):
    end = pyqtSignal(str)
    def __init__(self):
        super().__init__()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.showMaximized()
    p =prueba()
    p.end.connect(window.SystemWindow.SystemVerifRadioBtnWindow.on_ping_youtube)
    p.end.emit('youtube is up')
    sys.exit(app.exec_())