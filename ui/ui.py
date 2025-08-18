#!/usr/bin/env python3

import sys
import signal
import setproctitle
setproctitle.setproctitle("ui")
from PyQt5.QtWidgets import QApplication, QMainWindow, QTableWidgetItem
from PyQt5.QtCore import QTimer
from PyQt5 import uic
from PyQt5.QtGui import QImage, QPixmap

from functools import partial
import yaml 
import time 

app = None

from libs.widgets import *
from ros_manager import RosManager

def signal_handler(sig, frame):
    QApplication.quit()

class MyApp(QMainWindow):
    def __init__(self, type, test):
        form_class = uic.loadUiType("./forms/mainwindow.ui")[0]
        super(MyApp, self).__init__()
        self.ui = form_class()
        self.ui.setupUi(self)
        self.RM = RosManager(type, test)
        self.type = type

        self.last_image_update_time = 0
        self.last_displayed_image = None

        self.set_values()
        self.set_widgets()
        self.set_timers()

    def set_values(self):
        self.sig_in = False
        self.sig_in_viz = False
        self.state_list = {'temp': 0, 'prev': 0, 'target':0}
        self.state_string = ['Normal', 'Left Change', 'Right Change', 'Straight', 'Safe', 'Dangerous', 'Init', 'Emergency']
        self.signal_buttons = {self.ui.leftButton:1, self.ui.rightButton:2, self.ui.straightButton:3,self.ui.eButton:7}
        self.selfdriving_buttons = [self.ui.stopButton, self.ui.startButton]
        self.check_map = {self.ui.check1 : 1, self.ui.check2 : 2, self.ui.check3 : 3, self.ui.check4 : 4, self.ui.check5:5, self.ui.check6:6}
        self.radio_point = {self.ui.radioPlot1: 1, self.ui.radioPlot2: 2, self.ui.radioPlot3: 3,self.ui.radioPlot4: 4, self.ui.radioPlot5: 5, self.ui.radioPlot6: 6}

        self.ego_signal, self.target_signal = 0,0

    def set_widgets(self):
        self.rviz_widget = RvizWidget(self, self.type)
        self.rtt_graph = SpeedSubscriberWidget('#1a73eb', self)
        self.speed_graph = SpeedSubscriberWidget('#e23a2e', self)
        self.delay_graph = SpeedSubscriberWidget('#fc8c03', self)
        self.packet_size_graph = SpeedSubscriberWidget('#fbbf12',self)
        self.packet_rate_graph = SpeedSubscriberWidget('#279847',self)
        self.initUI()
    
    def set_timers(self):
        # 일반 UI 업데이트 (통신 성능, 상태)
        self.timer = QTimer(self)   
        self.timer.timeout.connect(self.updateUI)
        self.timer.start(30)  # 30ms

        # 실시간 이미지 업데이트를 위한 빠른 타이머
        self.image_timer = QTimer(self)
        self.image_timer.timeout.connect(self.update_image)
        self.image_timer.start(33)  # 33ms (~30 FPS)

        self.user_input_timer = QTimer(self)
        self.user_input_timer.timeout.connect(self.state_triggered)

        self.user_input_viz_timer = QTimer(self)
        self.user_input_viz_timer.timeout.connect(self.state_triggered_viz)
        

    def updateUI(self):
        """이미지를 제외한 UI 요소 업데이트"""
        self.comm_perform_update(self.RM.communication_performance)
        self.state_update(self.RM.signals)
    
    def update_image(self):
        """실시간 이미지 업데이트"""
        current_image = self.RM.get_latest_image()
        
        # 새로운 이미지가 있을 때만 업데이트
        if current_image is not None and current_image != self.last_displayed_image:
            try:
                # 스케일링을 빠르게 처리
                scaled_pixmap = current_image.scaled(
                    self.ui.cameraLabel.size(), 
                    aspectRatioMode=True,
                    transformMode=1  # Qt.FastTransformation for speed
                )
                self.ui.cameraLabel.setPixmap(scaled_pixmap)
                self.last_displayed_image = current_image
                
                # 디버그 출력 최소화
                # print("Image displayed at:", time.time())
            except Exception as e:
                print(f"Image display error: {e}")
    
    def comm_perform_update(self, communication_performance):
        self.ui.tableWidget.setStyleSheet('background-color: rgb(238, 238, 236);')
        self.ui.tableWidget.setItem(-1, 1, QTableWidgetItem(communication_performance['comulative_time']))
        self.ui.tableWidget.setItem(0, 1, QTableWidgetItem(communication_performance['distance']))
        self.ui.tableWidget.setItem(1, 1, QTableWidgetItem(communication_performance['rtt']))
        self.ui.tableWidget.setItem(2, 1, QTableWidgetItem(communication_performance['speed']))
        self.ui.tableWidget.setItem(3, 1, QTableWidgetItem(communication_performance['delay']))
        self.ui.tableWidget.setItem(4, 1, QTableWidgetItem(communication_performance['packet_size']))
        self.ui.tableWidget.setItem(5, 1, QTableWidgetItem(communication_performance['packet_rate']))
        self.rtt_graph.set_speed(communication_performance['rtt'])
        self.speed_graph.set_speed(communication_performance['speed'])
        self.delay_graph.set_speed(communication_performance['delay'])
        self.packet_size_graph.set_speed(communication_performance['packet_size'])
        self.packet_rate_graph.set_speed(communication_performance['packet_rate'])

    def state_update(self, signals):
        ego_signal = int(signals['ego'])
        target_signal = int(signals['target'])
        if ego_signal != self.ego_signal:
            self.ego_signal = ego_signal
            self.check_viz_timer()
        if target_signal != self.target_signal:
            self.target_signal = target_signal
            self.check_viz_timer()

        self.ui.egoLabel.setText(self.state_string[self.ego_signal])
        self.ui.targetLabel.setText(self.state_string[self.target_signal])
        self.ui.testCase.setText(self.RM.test_case)

    def click_new(self):
        for radio, value in self.radio_point.items():
            if radio.isChecked():
                point_num = value
        with open(f"./yaml/{self.type}_point.yaml", "r") as f:
            config = yaml.safe_load(f)

        config[str(point_num)] = {
            'point': self.RM.ego_pos
        }

        with open(f"./yaml/{self.type}_point.yaml", "w") as f:
            yaml.safe_dump(config, f)
        self.RM.publish_plot_point(self.RM.ego_pos)

    def click_load(self):
        for radio, value in self.radio_point.items():
            if radio.isChecked():
                point_num = value
        with open(f"./yaml/{self.type}_point.yaml", "r") as f:
            config = yaml.safe_load(f)
        
        point_config = config[str(point_num)]
        point = point_config['point']

        self.RM.publish_plot_point(point)

        
    def click_signal(self, value):
        self.RM.user_input[1] = value
        self.state = value
        self.check_timer()
        self.check_viz_timer()
    
    def click_selfdriving(self, value):
        self.RM.user_input[0] = value
        self.check_timer()
    
    def click_set(self):
        self.RM.user_input[2] = float(self.ui.velocityBox.value()/3.6)
        for checkbox, value in self.check_map.items():
            if checkbox.isChecked():
                self.RM.user_input[3] = value
                break
        self.check_timer()

    def check_timer(self):
        if not self.sig_in:
            self.sig_in = True
            self.user_input_timer.start(300)
            QTimer.singleShot(4000, self.stop_user_input_timer)
        else:
            self.stop_user_input_timer()
    
    def check_viz_timer(self):
        if not self.sig_in_viz:
            self.sig_in_viz = True
            self.user_input_viz_timer.start(1000)
            QTimer.singleShot(5000, self.stop_user_input_viz_timer)
    
    def stop_user_input_timer(self):
        self.sig_in = False
        self.RM.user_input[1] = 0
        self.user_input_timer.stop()
        self.RM.publish()

    def stop_user_input_viz_timer(self):
        self.sig_in_viz = False
        self.ego_signal = 0
        self.target_signal = 0
        self.ui.egoLabel.setStyleSheet("QLabel {background-color: white;}")
        self.ui.targetLabel.setStyleSheet("QLabel {background-color: white;}")
        self.user_input_viz_timer.stop()   

    def state_triggered(self):
        self.RM.publish()

    def state_triggered_viz(self):
        if self.ego_signal in [1,2,3]:
            self.ui.egoLabel.setStyleSheet("QLabel {background-color: yellow;}")
        if self.target_signal in [1,2,3]:
            self.ui.targetLabel.setStyleSheet("QLabel {background-color: yellow;}")
        if self.ego_signal == 4:
            self.ui.egoLabel.setStyleSheet("QLabel {background-color: green;}")
        if self.target_signal == 4:
            self.ui.targetLabel.setStyleSheet("QLabel {background-color: green;}")
        if self.ego_signal in [5,7]:
            self.ui.egoLabel.setStyleSheet("QLabel {background-color: red;}")
        if self.target_signal in [5,7]:
            self.ui.targetLabel.setStyleSheet("QLabel {background-color: red;}")

    def initUI(self):
        self.set_conntection()
        self.ui.rvizLayout.addWidget(self.rviz_widget)
        self.ui.RTTLayout.addWidget(self.rtt_graph)
        self.ui.SpeedLayout.addWidget(self.speed_graph)
        self.ui.DelayLayout.addWidget(self.delay_graph)
        self.ui.PacketSizeLayout.addWidget(self.packet_size_graph)
        self.ui.PacketRateLayout.addWidget(self.packet_rate_graph)

    def set_conntection(self):
        for button, value in self.signal_buttons.items():
            button.clicked.connect(partial(self.click_signal,int(value)))
        for i in range(0,2):
            self.selfdriving_buttons[i].clicked.connect(partial(self.click_selfdriving, int(i)))
        self.ui.setButton.clicked.connect(self.click_set)
        self.ui.newButton.clicked.connect(self.click_new)
        self.ui.loadButton.clicked.connect(self.click_load)
    

def main():
    app = QApplication(sys.argv)
    type = sys.argv[1]
    test = int(sys.argv[2])
    ex = MyApp(type, test)
    ex.show()
    signal.signal(signal.SIGINT, signal_handler)
    app.exec_()

if __name__ == '__main__':
    main()