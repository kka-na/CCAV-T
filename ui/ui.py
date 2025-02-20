#!/usr/bin/env python3

import sys
import signal
import setproctitle
setproctitle.setproctitle("ui")
from PyQt5.QtWidgets import QApplication, QMainWindow, QTableWidgetItem
from PyQt5.QtCore import QTimer
from PyQt5 import uic
from functools import partial

app = None

from libs.widgets import *
from ros_manager import RosManager

def signal_handler(sig, frame):
    QApplication.quit()

class MyApp(QMainWindow):
    def __init__(self, type):
        form_class = uic.loadUiType("./forms/mainwindow.ui")[0]
        super(MyApp, self).__init__()
        self.ui = form_class()
        self.ui.setupUi(self)
        self.RM = RosManager(type)
        self.type = type

        self.set_values()
        self.set_widgets()
        self.set_timers()

    def set_values(self):
        self.sig_in = False

        self.state_list = {'temp': 0, 'prev': 0, 'target':0}
        self.state_string = ['Normal', 'Left Change', 'Right Change', 'Straight', 'Safe', 'Dangerous']
        self.signal_buttons = ['', self.ui.leftButton, self.ui.rightButton, self.ui.straightButton, self.ui.eButton]
        self.selfdriving_buttons = [self.ui.stopButton, self.ui.startButton]

    def set_widgets(self):
        self.rviz_widget = RvizWidget(self, self.type)
        self.rtt_graph = SpeedSubscriberWidget('#1a73eb', self)
        self.speed_graph = SpeedSubscriberWidget('#e23a2e', self)
        self.packet_size_graph = SpeedSubscriberWidget('#fbbf12',self)
        self.packet_rate_graph = SpeedSubscriberWidget('#279847',self)
        self.initUI()
    
    def set_timers(self):
        self.timer = QTimer(self)   
        self.timer.timeout.connect(self.updateUI)
        self.timer.start(100)

        self.user_input_timer = QTimer(self)
        self.user_input_timer.timeout.connect(self.state_triggered)
        self.user_input_timer.start(200)
        

    def updateUI(self):
        self.comm_perform_update(self.RM.communication_performance)
        self.state_update(self.RM.states)
    
    def comm_perform_update(self, communication_performance):
        self.ui.tableWidget.setStyleSheet('background-color: rgb(238, 238, 236);')
        self.ui.tableWidget.setItem(-1, 1, QTableWidgetItem(communication_performance['comulative_time']))
        self.ui.tableWidget.setItem(0, 1, QTableWidgetItem(communication_performance['distance']))
        self.ui.tableWidget.setItem(1, 1, QTableWidgetItem(communication_performance['rtt']))
        self.ui.tableWidget.setItem(2, 1, QTableWidgetItem(communication_performance['speed']))
        self.ui.tableWidget.setItem(3, 1, QTableWidgetItem(communication_performance['packet_size']))
        self.ui.tableWidget.setItem(4, 1, QTableWidgetItem(communication_performance['packet_rate']))
        self.rtt_graph.set_speed(communication_performance['rtt'])
        self.speed_graph.set_speed(communication_performance['speed'])
        self.packet_size_graph.set_speed(communication_performance['packet_size'])
        self.packet_rate_graph.set_speed(communication_performance['packet_rate'])

    def state_update(self, states):
        ego_state = int(self.state_list['temp'])
        target_state = int(states['target'])

        if self.state_list['target'] != target_state and target_state != 0:
            self.state_list['target'] = target_state
            if self.type == 'target':
                self.click_state(target_state)
            else:
                self.check_viz_timer()

        self.ui.egoLabel.setText(self.state_string[ego_state])
        self.ui.targetLabel.setText(self.state_string[self.state_list['target']])
        
    def click_signal(self, value):
        self.RM.user_input[1] = value
        self.state = value
        self.check_timer()
    
    def click_selfdriving(self, value):
        self.RM.user_input[0] = value
        self.check_timer()
    
    def click_set(self):
        self.RM.user_input[2] = float(self.ui.velocityBox.value()/3.6)
        self.check_timer()

    def check_timer(self):
        if not self.sig_in:
            self.sig_in = True
            self.user_input_timer.start(200)
            QTimer.singleShot(1600, self.stop_user_input_timer)
        else:
            self.stop_user_input_timer()
    
    def stop_user_input_timer(self):
        self.sig_in = False
        self.RM.user_input[1] = 0
        self.user_input_timer.stop()
        self.RM.publish()

    def state_triggered(self):
        self.RM.publish()

    def stop_user_input_viz_timer(self):
        self.state = 0
        self.target_state = 0
        self.user_input_viz_timer.stop()        

    def initUI(self):
        self.set_conntection()
        self.ui.rvizLayout.addWidget(self.rviz_widget)
        self.ui.RTTLayout.addWidget(self.rtt_graph)
        self.ui.SpeedLayout.addWidget(self.speed_graph)
        self.ui.PacketSizeLayout.addWidget(self.packet_size_graph)
        self.ui.PacketRateLayout.addWidget(self.packet_rate_graph)

    def set_conntection(self):
        for i in range(1,5):
            self.signal_buttons[i].clicked.connect(partial(self.click_signal, int(i)))
        for i in range(0,2):
            self.selfdriving_buttons[i].clicked.connect(partial(self.click_selfdriving, int(i)))
        self.ui.setButton.clicked.connect(self.click_set)
def main():
    app = QApplication(sys.argv)
    type = sys.argv[1]
    ex = MyApp(type)
    ex.show()
    signal.signal(signal.SIGINT, signal_handler)
    app.exec_()

if __name__ == '__main__':
    main()
