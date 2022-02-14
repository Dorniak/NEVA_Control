from PyQt5 import QtCore, QtGui
from PyQt5.QtWidgets import (
    QWidget, QGridLayout,
    QPushButton, QLabel,
    QCheckBox, QRadioButton,
    QLineEdit, QShortcut
)
from PyQt5.QtCore import Qt
from control.gui.utils import (
    Slider, GooglePingThread,
    MessageDialog, YahooPingThread,
    GmailPingThread, youtubePingThread,
    FacebookPingThread
)
from os import path


class SystemVerifRadioBtnWindow(QWidget):
    def __init__(self, parent):
        super().__init__()
        self.Board5_color_label_status = False
        self.Board6_color_label_status = False
        self.Board7_color_label_status = False
        self.Brake_color_label_status = False
        self.CAN_color_label_status = False
        self.SetupUI()

    def SetupUI(self):

        self.layout = QGridLayout(self)
        self.background = QLabel(self)
        self.background.setStyleSheet("border: 1px solid black;")

        self.heading = QLabel('System Verif', self)
        self.heading.setAlignment(Qt.AlignCenter)
        self.heading.setFont(QtGui.QFont('Arial', 12, weight=QtGui.QFont.Bold))

        self.Board5_color_label = QLabel(self)
        self.Board5_color_label.setStyleSheet(
            "border: 1px solid black;border-radius: 10px; min-height: 18px;min-width: 18px;background-color: red")

        self.Board6_color_label = QLabel(self)
        self.Board6_color_label.setStyleSheet(
            "border: 1px solid black;border-radius: 10px; min-height: 18px;min-width: 18px;background-color: red")

        self.Board7_color_label = QLabel(self)
        self.Board7_color_label.setStyleSheet(
            "border: 1px solid black;border-radius: 10px; min-height: 18px;min-width: 18px;background-color: red")

        self.Brake_color_label = QLabel(self)
        self.Brake_color_label.setStyleSheet(
            "border: 1px solid black;border-radius: 10px; min-height: 18px;min-width: 18px;background-color: red")

        self.CAN_color_label = QLabel(self)
        self.CAN_color_label.setStyleSheet(
            "border: 1px solid black;border-radius: 10px; min-height: 18px;min-width: 18px;background-color: red")

        self.GooglePingThread = GooglePingThread(self)
        self.GooglePingThread.end.connect(self.on_ping_google)
        self.GooglePingThread.running = True
        self.GooglePingThread.start()

        # self.YoutubePingThread = youtubePingThread(self)
        # self.YoutubePingThread.end.connect(self.on_ping_youtube)
        # self.YoutubePingThread.running = True
        # self.YoutubePingThread.start()

        self.FacebookPingThread = FacebookPingThread(self)
        self.FacebookPingThread.end.connect(self.on_ping_facebook)
        self.FacebookPingThread.running = True
        self.FacebookPingThread.start()

        self.GmailPingThread = GmailPingThread(self)
        self.GmailPingThread.end.connect(self.on_ping_gmail)
        self.GmailPingThread.running = True
        self.GmailPingThread.start()

        # self.YahooPingThread = YahooPingThread(self)
        # self.YahooPingThread.end.connect(self.callback_CAN_signal)
        # self.YahooPingThread.running = True
        # self.YahooPingThread.start()

        self.Board5_label = QLabel('Board 5', self)
        self.Board6_label = QLabel('Board 6', self)
        self.Board7_label = QLabel('Board 7', self)
        self.Brake_label = QLabel('Brake', self)
        self.CAN_label = QLabel('CAN', self)

        self.layout.addWidget(self.background, 0, 0, 20, 20)
        self.layout.addWidget(self.heading, 1, 2, 2, 2)
        self.layout.addWidget(self.Board5_color_label, 4, 1)
        self.layout.addWidget(self.Board5_label, 4, 2)
        self.layout.addWidget(self.Board6_color_label, 7, 1)
        self.layout.addWidget(self.Board6_label, 7, 2)
        self.layout.addWidget(self.Board7_color_label, 10, 1)
        self.layout.addWidget(self.Board7_label, 10, 2)
        self.layout.addWidget(self.Brake_color_label, 13, 1)
        self.layout.addWidget(self.Brake_label, 13, 2)
        self.layout.addWidget(self.CAN_color_label, 16, 1)
        self.layout.addWidget(self.CAN_label, 16, 2)

    def on_ping_google(self, status):
        if status == 'google is up':
            self.Board5_color_label.setStyleSheet(
                "border: 1px solid black;border-radius: 10px; min-height: 18px;min-width: 18px;background-color: green")
        if status == 'google is down':
            self.Board5_color_label.setStyleSheet(
                "border: 1px solid black;border-radius: 10px; min-height: 18px;min-width: 18px;background-color: red")

    def on_ping_gmail(self, status):
        if status == 'gmail is up':
            self.Board6_color_label.setStyleSheet(
                "border: 1px solid black;border-radius: 10px; min-height: 18px;min-width: 18px;background-color: green")
        if status == 'gmail is down':
            self.Board6_color_label.setStyleSheet(
                "border: 1px solid black;border-radius: 10px; min-height: 18px;min-width: 18px;background-color: red")

    def on_ping_facebook(self, status):
        if status == 'facebook is up':
            self.Board7_color_label.setStyleSheet(
                "border: 1px solid black;border-radius: 10px; min-height: 18px;min-width: 18px;background-color: green")
        if status == 'facebook is down':
            self.Board7_color_label.setStyleSheet(
                "border: 1px solid black;border-radius: 10px; min-height: 18px;min-width: 18px;background-color: red")

    def callback_brake_signal(self, status):
        if status:
            self.Brake_color_label.setStyleSheet(
                "border: 1px solid black;border-radius: 10px; min-height: 18px;min-width: 18px;background-color: green")
        else:
            self.Brake_color_label.setStyleSheet(
                "border: 1px solid black;border-radius: 10px; min-height: 18px;min-width: 18px;background-color: red")

    def callback_CAN_signal(self, status):
        if status:
            self.CAN_color_label.setStyleSheet(
                "border: 1px solid black;border-radius: 10px; min-height: 18px;min-width: 18px;background-color: green")
        else:
            self.CAN_color_label.setStyleSheet(
                "border: 1px solid black;border-radius: 10px; min-height: 18px;min-width: 18px;background-color: red")


class SystemVerifDriveCheckboxWindow(QWidget):
    def __init__(self, parent):
        super().__init__()
        self.SetupUI()

    def SetupUI(self):

        self.layout = QGridLayout(self)
        self.background = QLabel(self)
        self.background.setStyleSheet("border: 1px solid black;")

        self.checkbox_M_drive = QCheckBox('M_drive')
        self.checkbox_M_drive.setStyleSheet("QCheckBox"
                                            "{spacing : 20px; color: black; font-size:18px; font-family: Arial};"
                                            "QCheckBox::indicator { width: 25px; height: 25px; spacing : 20px};")
        self.checkbox_M_drive.stateChanged.connect(self.on_checkbox_M_drive)

        self.checkbox_IP_drive = QCheckBox('IP_drive')
        self.checkbox_IP_drive.setStyleSheet("QCheckBox"
                                             "{spacing : 20px; color: black; font-size:18px; font-family: Arial};"
                                             "QCheckBox::indicator { width: 25px; height: 25px; spacing : 20px};")
        self.checkbox_IP_drive.stateChanged.connect(self.on_checkbox_IP_drive)

        self.checkbox_Auto_drive = QCheckBox('Auto_drive')
        self.checkbox_Auto_drive.setStyleSheet("QCheckBox"
                                               "{spacing : 20px; color: black; font-size:18px; font-family: Arial};"
                                               "QCheckBox::indicator { width: 25px; height: 25px; spacing : 20px};")
        self.checkbox_Auto_drive.stateChanged.connect(self.on_checkbox_Auto_drive)

        self.checkbox_Route_drive = QCheckBox('Route_drive')
        self.checkbox_Route_drive.setStyleSheet("QCheckBox"
                                                "{spacing : 20px; color: black; font-size:18px; font-family: Arial};"
                                                "QCheckBox::indicator { width: 25px; height: 25px; spacing : 20px};")
        self.checkbox_Route_drive.stateChanged.connect(self.on_checkbox_Route_drive)

        self.layout.addWidget(self.background, 0, 0, 20, 20)
        self.layout.addWidget(self.checkbox_M_drive, 4, 1, 2, 2)
        self.layout.addWidget(self.checkbox_IP_drive, 7, 1, 2, 2)
        self.layout.addWidget(self.checkbox_Auto_drive, 10, 1, 2, 2)
        self.layout.addWidget(self.checkbox_Route_drive, 13, 1, 2, 2)

    def on_checkbox_M_drive(self):
        if self.checkbox_M_drive.isChecked():
            self.checkbox_IP_drive.setChecked(False)
            self.checkbox_Auto_drive.setChecked(False)
            self.checkbox_Route_drive.setChecked(False)

    def on_checkbox_IP_drive(self):
        if self.checkbox_IP_drive.isChecked():
            self.checkbox_M_drive.setChecked(False)
            self.checkbox_Auto_drive.setChecked(False)
            self.checkbox_Route_drive.setChecked(False)

    def on_checkbox_Auto_drive(self):
        if self.checkbox_Auto_drive.isChecked():
            self.checkbox_M_drive.setChecked(False)
            self.checkbox_IP_drive.setChecked(False)
            self.checkbox_Route_drive.setChecked(False)

    def on_checkbox_Route_drive(self):
        if self.checkbox_Route_drive.isChecked():
            self.checkbox_M_drive.setChecked(False)
            self.checkbox_IP_drive.setChecked(False)
            self.checkbox_Auto_drive.setChecked(False)


class SystemVerifSpeedLimitCheckboxWindow(QWidget):
    def __init__(self, parent):
        super().__init__()
        self.SetupUI()

    def SetupUI(self):

        self.layout = QGridLayout(self)
        self.background = QLabel(self)
        self.background.setStyleSheet("border: 1px solid black;border-right: 1px solid white;")

        self.Speed_limit_line_edit = QLineEdit('15', self)
        self.Speed_limit_line_edit.setFont(QtGui.QFont("Arial", 16, weight=QtGui.QFont.Bold))
        self.Speed_limit_line_edit.setFixedHeight(30)
        self.Speed_limit_line_edit.setFixedWidth(50)
        self.Speed_limit_line_edit.setMaxLength(3)
        self.Speed_limit_line_edit.setAlignment(QtCore.Qt.AlignCenter)
        self.Speed_limit_line_edit.setValidator(QtGui.QIntValidator())

        self.Speed_limit_label = QLabel('Speed limit', self)
        self.Speed_limit_label.setFont(QtGui.QFont('Arial', 12, weight=QtGui.QFont.Bold))

        self.onsafety_btn_label = QLabel('On safety button', self)

        self.checkbox_Release_all = QCheckBox('Release_all')
        self.checkbox_Release_all.setStyleSheet("QCheckBox"
                                                "{spacing : 20px; color: black; font-size:18px; font-family: Arial};"
                                                "QCheckBox::indicator { width: 25px; height: 25px; spacing : 20px};")
        self.checkbox_Release_all.setChecked(True)
        self.checkbox_Release_all.stateChanged.connect(self.on_checkbox_Release_all)

        self.checkbox_Brake_completely = QCheckBox('Brake_completely')
        self.checkbox_Brake_completely.setStyleSheet("QCheckBox"
                                                     "{spacing : 20px; color: black; font-size:18px; font-family: Arial};"
                                                     "QCheckBox::indicator { width: 25px; height: 25px; spacing : 20px};")
        self.checkbox_Brake_completely.stateChanged.connect(self.on_checkbox_Brake_completely)

        # self.Logo = QLabel(self)
        # pixmap = QtGui.QPixmap('./resources/Neva2.png')
        # self.Logo.setPixmap(pixmap)
        # self.Logo.setAlignment(QtCore.Qt.AlignCenter)
        # self.Logo.setFixedSize(20,20)
        # self.Logo.resize(150, 40)

        self.layout.addWidget(self.background, 0, 0, 20, 20)
        self.layout.addWidget(self.Speed_limit_line_edit, 2, 1, 2, 2)
        # self.layout.addWidget(self.Logo,5,12,2,2)
        self.layout.addWidget(self.Speed_limit_label, 2, 2, 2, 2)
        self.layout.addWidget(self.onsafety_btn_label, 4, 2, 2, 2)
        self.layout.addWidget(self.checkbox_Release_all, 7, 1, 2, 2)
        self.layout.addWidget(self.checkbox_Brake_completely, 10, 1, 2, 2)

    def on_checkbox_Release_all(self):
        if self.checkbox_Release_all.isChecked():
            self.checkbox_Brake_completely.setChecked(False)

    def on_checkbox_Brake_completely(self):
        if self.checkbox_Brake_completely.isChecked():
            self.checkbox_Release_all.setChecked(False)


class SystemVerifValuesWindow(QWidget):
    def __init__(self, parent):
        super().__init__()
        self.SetupUI()

    def SetupUI(self):
        self.layout = QGridLayout(self)
        self.background = QLabel(self)
        self.background.setStyleSheet("border: 1px solid black;")

        self.Volante_label = QLabel('Volante', self)
        self.Volante_label.setFont(QtGui.QFont('Arial', 14))
        self.Volante_label.setAlignment(Qt.AlignCenter)
        self.Volante_label.setStyleSheet("border-left: 1px solid black;")

        self.Volante_no_label = QLabel('0', self)
        self.Volante_no_label.setFont(QtGui.QFont('Arial', 16, weight=QtGui.QFont.Bold))

        self.Velocidad_label = QLabel('Velocidad', self)
        self.Velocidad_label.setFont(QtGui.QFont('Arial', 14))

        self.Velocidad_label_no = QLabel('0', self)
        self.Velocidad_label_no.setFont(QtGui.QFont('Arial', 16, weight=QtGui.QFont.Bold))

        self.freno_label = QLabel('Freno', self)
        self.freno_label.setFont(QtGui.QFont('Arial', 14))
        self.freno_label.setAlignment(Qt.AlignCenter)

        self.freno_label_no = QLabel('0', self)
        self.freno_label_no.setFont(QtGui.QFont('Arial', 16, weight=QtGui.QFont.Bold))
        self.freno_label_no.setAlignment(Qt.AlignCenter)

        self.p_emergencia_label = QLabel('P Emergencia', self)
        self.p_emergencia_label.setFont(QtGui.QFont('Arial', 14))
        self.p_emergencia_label.setAlignment(Qt.AlignCenter)

        self.p_emergencia_label_no = QLabel('False', self)
        self.p_emergencia_label_no.setFont(QtGui.QFont('Arial', 16, weight=QtGui.QFont.Bold))
        self.p_emergencia_label_no.setAlignment(Qt.AlignCenter)

        self.marcha_label = QLabel('Marcha', self)
        self.marcha_label.setFont(QtGui.QFont('Arial', 14))
        self.marcha_label.setAlignment(Qt.AlignCenter)

        self.marcha_label_no = QLabel('?', self)
        self.marcha_label_no.setFont(QtGui.QFont('Arial', 16, weight=QtGui.QFont.Bold))

        self.layout.addWidget(self.background, 0, 0, 20, 20)
        self.layout.addWidget(self.Volante_label, 4, 0, 1, 3)
        self.layout.addWidget(self.Volante_no_label, 14, 1, 2, 2)
        self.layout.addWidget(self.Velocidad_label, 4, 4, 1, 3)
        self.layout.addWidget(self.Velocidad_label_no, 14, 5, 2, 2)
        self.layout.addWidget(self.freno_label, 4, 8, 1, 3)
        self.layout.addWidget(self.freno_label_no, 14, 8, 2, 3)
        self.layout.addWidget(self.p_emergencia_label, 4, 12, 1, 3)
        self.layout.addWidget(self.p_emergencia_label_no, 14, 12, 2, 3)
        self.layout.addWidget(self.marcha_label, 4, 16, 1, 3)
        self.layout.addWidget(self.marcha_label_no, 14, 17, 2, 2)


class ShowNevaImageWidget(QWidget):
    def __init__(self, parent):
        super().__init__()
        self.SetupUI()

    def SetupUI(self):
        self.layout = QGridLayout(self)
        self.BackBtn = QPushButton(self)
        self.BackBtn.setStyleSheet("background-color : white; border: none;")
        path_resetbtn = path.dirname(path.realpath(__file__))
        self.BackBtn.setIcon(QtGui.QIcon(path_resetbtn+'/resources/resetbtn.png'))
        self.BackBtn.setIconSize(QtCore.QSize(80, 80))
        self.layout.addWidget(self.BackBtn, 1, 1)


class ShowNevaLogoWidget(QWidget):
    def __init__(self, parent):
        super().__init__()
        self.SetupUI()

    def SetupUI(self):
        self.layout = QGridLayout(self)
        self.Logo = QLabel(self)
        path_neva2 = path.dirname(path.realpath(__file__))
        pixmap = QtGui.QPixmap(path_neva2 + '/resources/Neva2.png')
        self.Logo.setPixmap(pixmap)
        self.Logo.setAlignment(QtCore.Qt.AlignCenter)
        self.layout.addWidget(self.Logo, 0, 0, 1, 1)


class SystemNevaWindow(QWidget):
    def __init__(self, parent, SystemVerifSpeedLimitCheckboxWindow, ShowNevaImageWidget):
        super().__init__()
        self.setStyleSheet("background-color: rgb(255, 255, 255)")
        self.SystemVerifSpeedLimitCheckboxWindow = SystemVerifSpeedLimitCheckboxWindow
        self.ShowNevaImageWidget = ShowNevaImageWidget
        self.SetupUI()

    def SetupUI(self):
        self.background = QLabel(self)
        self.background.setStyleSheet("border: 1px solid black;")
        self.layout = QGridLayout(self)

        self.heading = QLabel('NEVA', self)
        self.heading.setStyleSheet("color:black; font-size:18px; font-family: Arial")

        self.checkbox_Volante = QCheckBox('Volante')
        self.checkbox_Volante.setStyleSheet("QCheckBox"
                                            "{spacing : 20px; color: black; font-size:18px; font-family: Arial};"
                                            "QCheckBox::indicator { width: 25px; height: 25px; spacing : 20px};")
        self.checkbox_Volante.setChecked(True)

        self.VolanteSlider = Slider(QtCore.Qt.Horizontal, self)
        self.VolanteSlider.setRange(-540, 540)
        self.VolanteSlider.setMinimum(-540)
        self.VolanteSlider.setMaximum(540)
        self.VolanteSlider.setValue(0)
        self.VolanteSlider.setSingleStep(5)
        self.VolanteSlider.valueChanged.connect(self.on_volante_slider)

        self.Volante_reset_button = QPushButton('Reset')
        self.Volante_reset_button.setFont(QtGui.QFont('Arial', 12, weight=QtGui.QFont.Bold))
        self.Volante_reset_button.setStyleSheet(
            'QPushButton {background-color: white; color: black; border-radius:4px; border: 1px solid black;padding:5px 0}'
            'QPushButton:hover{background-color:lightgray;}')
        self.Volante_reset_button.clicked.connect(self.on_volante_reset_button)

        self.Volante_value_label = QLabel('0', self)
        self.Volante_value_label.setStyleSheet("color:black; font-size:18px; font-family: Arial")
        self.Volante_value_label.setAlignment(Qt.AlignCenter)

        self.shortcut_key_up = QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Up), self)
        self.shortcut_key_up.activated.connect(self.on_up_key)

        self.shortcut_key_down = QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Down), self)
        self.shortcut_key_down.activated.connect(self.on_down_key)

        self.shortcut_key_left = QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Left), self)
        self.shortcut_key_left.activated.connect(self.on_left_key)

        self.shortcut_key_right = QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Right), self)
        self.shortcut_key_right.activated.connect(self.on_right_key)

        self.shortcut_key_space = QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Space), self)
        self.shortcut_key_space.activated.connect(self.on_reset_all)

        self.checkbox_Velocidad = QCheckBox('Velocidad')
        self.checkbox_Velocidad.setStyleSheet("QCheckBox"
                                              "{spacing : 20px; color: black; font-size:18px; font-family: Arial};"
                                              "QCheckBox::indicator { width: 25px; height: 25px; spacing : 20px};")
        self.checkbox_Velocidad.setChecked(True)

        self.VelocidadSlider = Slider(QtCore.Qt.Horizontal, self)
        self.VelocidadSlider.setRange(0, 100)
        self.VelocidadSlider.setValue(0)
        self.VelocidadSlider.setMinimum(0)
        self.VelocidadSlider.setMaximum(100)
        self.VelocidadSlider.setSingleStep(0)
        self.VelocidadSlider.valueChanged.connect(self.on_Velocidad_slider)

        self.Velocidad_reset_button = QPushButton('Reset')
        self.Velocidad_reset_button.setFont(QtGui.QFont('Arial', 12, weight=QtGui.QFont.Bold))
        self.Velocidad_reset_button.setStyleSheet(
            'QPushButton {background-color: white; color: black; border-radius:4px; border: 1px solid black;padding:5px 0}'
            'QPushButton:hover{background-color:lightgray;}')
        self.Velocidad_reset_button.clicked.connect(self.on_Velocidad_reset_button)

        self.Velocidad_value_label = QLabel('0 km/h', self)
        self.Velocidad_value_label.setStyleSheet("color:black; font-size:18px; font-family: Arial")
        self.Velocidad_value_label.setAlignment(Qt.AlignCenter)

        self.ShowNevaImageWidget.BackBtn.clicked.connect(self.on_reset_all)

        self.layout.addWidget(self.background, 1, 1, 20, 20)
        self.layout.addWidget(self.heading, 2, 2)
        self.layout.addWidget(self.checkbox_Volante, 6, 2, 2, 2)
        self.layout.addWidget(self.VolanteSlider, 6, 5, 2, 8)
        self.layout.addWidget(self.Volante_reset_button, 6, 13, 2, 2)
        self.layout.addWidget(self.Volante_value_label, 6, 15, 2, 1)
        self.layout.addWidget(self.checkbox_Velocidad, 12, 2, 2, 2)
        self.layout.addWidget(self.VelocidadSlider, 12, 5, 2, 8)
        self.layout.addWidget(self.Velocidad_reset_button, 12, 13, 2, 2)
        self.layout.addWidget(self.Velocidad_value_label, 12, 15, 2, 2)

    def on_volante_slider(self):
        slider_val = self.VolanteSlider.value()
        self.Volante_value_label.setText(str(slider_val))

    def on_up_key(self):
        value = self.VelocidadSlider.value()
        if value < self.VelocidadSlider.maximum():
            self.VelocidadSlider.setValue(value + 1)

    def on_down_key(self):
        value = self.VelocidadSlider.value()
        if value > self.VelocidadSlider.minimum():
            self.VelocidadSlider.setValue(value - 1)

    def on_left_key(self):
        value = self.VolanteSlider.value()
        if value > self.VolanteSlider.minimum():
            self.VolanteSlider.setValue(value - 1)

    def on_right_key(self):
        value = self.VolanteSlider.value()
        if value < self.VolanteSlider.maximum():
            self.VolanteSlider.setValue(value + 1)

    def on_volante_reset_button(self):
        if self.SystemVerifSpeedLimitCheckboxWindow.checkbox_Release_all.isChecked():
            self.checkbox_Volante.setChecked(False)
        if self.SystemVerifSpeedLimitCheckboxWindow.checkbox_Brake_completely.isChecked():
            self.checkbox_Volante.setChecked(False)

    def on_Velocidad_slider(self):
        slider_val = self.VelocidadSlider.value()
        value = str(slider_val) + ' km/h'
        self.Velocidad_value_label.setText(value)

    def on_Velocidad_reset_button(self):
        if self.SystemVerifSpeedLimitCheckboxWindow.checkbox_Release_all.isChecked():
            self.checkbox_Velocidad.setChecked(False)
        if self.SystemVerifSpeedLimitCheckboxWindow.checkbox_Brake_completely.isChecked():
            self.VelocidadSlider.setValue(0)
            self.Velocidad_value_label.setText('0 km/h')

    def on_reset_all(self):
        self.on_volante_reset_button()
        self.on_Velocidad_reset_button()
