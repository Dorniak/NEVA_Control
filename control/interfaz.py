import sys
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow

from control.gui.gui_interface import Ui_MainWindow

import rclpy
from rclpy.node import Node

import threading

from neva_msg.msg import Status
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Float64, Bool


class First_window(QMainWindow):
    def __init__(self, parent=None):
        super(First_window, self).__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)


class NEVA_GUI(Node):
    def __init__(self, mainwindow=None, name: str = 'Unknown Interface'):
        super().__init__(node_name=name, namespace='interface', start_parameter_services=True,
                         allow_undeclared_parameters=False, automatically_declare_parameters_from_overrides=True)

        self.window = mainwindow
        self.window.ui.id.setText('NEVA')

        self.sub = self.create_subscription(Status, '/NEVA/status', self.subscriber_status,
                                            qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_b_volante = self.create_publisher(msg_type=Bool, topic='/NEVA/b_direccion',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_b_velocidad = self.create_publisher(msg_type=Bool, topic='/NEVA/b_velocidad',
                                                     qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_volante = self.create_publisher(msg_type=Float64, topic='/NEVA/direccion',
                                                 qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_velocidad = self.create_publisher(msg_type=Float64, topic='/NEVA/velocidad',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer = self.create_timer(1 / 10, self.publish_Control)

    def publish_Control(self):
        self.pub_b_volante.publish(Bool(data=self.window.ui.b_volante.isChecked()))
        self.pub_b_velocidad.publish(Bool(data=self.window.ui.b_velocidad.isChecked()))
        self.pub_volante.publish(Float64(data=float(self.window.ui.LateralSlider.value())))
        self.pub_velocidad.publish(Float64(data=float(self.window.ui.LongitudinalSlider.value())))

    def subscriber_status(self, msg):
        self.window.ui.velocidad_real.setText(str(round(msg.velocidad_real, 1)))
        self.window.ui.velocidad_real_2.setText(str(round(msg.velocidad_real, 1)))

        self.window.ui.direccion_real.setText(str(round(msg.direccion_real, 1)))
        self.window.ui.direccion_real_2.setText(str(round(msg.direccion_real, 1)))

        self.window.ui.freno_real.setText(str(round(msg.freno_real)) + ' %')

        self.window.ui.marcha_real.setText(str(msg.marcha_real))

        self.window.ui.velocidad_target.setText(str(round(msg.velocidad_target, 1)))

        self.window.ui.volante_target.setText(str(round(msg.volante_target, 1)))

        self.window.ui.b_velocidad_target.setText('TRUE' if msg.b_velocidad else 'FALSE')

        self.window.ui.b_volante_target.setText('TRUE' if msg.b_volante else 'FALSE')

        self.window.ui.parada_target.setText('TRUE' if msg.parada_emergencia_request else 'FALSE')

        self.window.ui.parada_real.setText('TRUE' if msg.parada_emergencia else 'FALSE')


def startthread(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)
    gui = First_window()
    node = NEVA_GUI(mainwindow=gui, name='Status_Interface')
    gui.show()
    spin = threading.Thread(target=startthread, kwargs=dict(Node=node), daemon=True, name='Thread publisher')
    spin.start()
    app.exec_()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
