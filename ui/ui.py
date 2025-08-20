#!/usr/bin/env python3

import sys
import signal
import setproctitle
setproctitle.setproctitle("ui")
from PyQt5.QtWidgets import QApplication, QMainWindow, QTableWidgetItem
from PyQt5.QtCore import QTimer, pyqtSlot
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

        # 성능 모니터링 변수
        self.last_image_update_time = 0
        self.last_displayed_image = None
        self.image_update_count = 0
        self.fps_display_timer = 0

        self.reject_once = False

        self.set_values()
        self.set_widgets()
        self.set_timers()

    def set_values(self):
        self.sig_in = False
        self.state = 0
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
        self.rtt_graph = SpeedSubscriberWidget('#1a73eb', 0, 2000, 'ms', self)
        self.speed_graph = SpeedSubscriberWidget('#e23a2e', 0, 0.05, 'mbps', self)
        self.delay_graph = SpeedSubscriberWidget('#fc8c03', 0, 1500, 'ms', self)
        self.packet_size_graph = SpeedSubscriberWidget('#fbbf12',0, 500, 'byte', self)
        self.packet_rate_graph = SpeedSubscriberWidget('#279847', 0, 100, '%', self)
        self.initUI()
    
    def set_timers(self):
        # 일반 UI 업데이트 (통신 성능, 상태) - 빈도 낮춤
        self.timer = QTimer(self)   
        self.timer.timeout.connect(self.updateUI)
        self.timer.start(100)  # 100ms (10fps) - 성능 향상을 위해 더 낮춤

        # 실시간 이미지 업데이트 전용 타이머 - 최고 빈도
        self.image_timer = QTimer(self)
        self.image_timer.timeout.connect(self.update_image)
        self.image_timer.start(10)  # 10ms (100fps 시도) - 가능한 한 빠르게

        self.user_input_timer = QTimer(self)
        self.user_input_timer.timeout.connect(self.state_triggered)

        self.user_input_viz_timer = QTimer(self)
        self.user_input_viz_timer.timeout.connect(self.state_triggered_viz)

    def updateUI(self):
        """이미지를 제외한 UI 요소 업데이트 (빈도 낮음)"""
        self.comm_perform_update(self.RM.communication_performance)
        self.state_update(self.RM.signals)
    
    @pyqtSlot()
    def update_image(self):
        """고성능 이미지 업데이트 - 슬롯 최적화"""
        current_image = self.RM.get_latest_image()
        
        # 새로운 이미지가 있을 때만 업데이트
        if current_image is not None and current_image is not self.last_displayed_image:
            try:
                # 직접 픽스맵 설정 (추가 처리 없음)
                self.ui.cameraLabel.setPixmap(current_image)
                self.last_displayed_image = current_image
                self.image_update_count += 1
                    
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
        self.rtt_graph.set_speed(float(communication_performance['rtt']))
        self.speed_graph.set_speed(float(communication_performance['speed']))
        self.delay_graph.set_speed(float(communication_performance['delay']))
        self.packet_size_graph.set_speed(float(communication_performance['packet_size']))
        self.packet_rate_graph.set_speed(float(communication_performance['packet_rate']))

    def state_update(self, signals):
        ego_signal = int(signals['ego'])
        target_signal = int(signals['target'])
        if target_signal == 5:
            self.reject_once = True
        if ego_signal != self.ego_signal:
            self.ego_signal = ego_signal
            self.check_viz_timer()
        if target_signal != self.target_signal:
            self.target_signal = target_signal
            if self.reject_once and target_signal == 0:
                self.click_signal(self.state)
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
        # 모든 signal button 스타일 리셋
        for button in self.signal_buttons.keys():
            button.setStyleSheet("")
        
        # 눌린 버튼만 초록색으로 설정
        for button, button_value in self.signal_buttons.items():
            if button_value == value:
                button.setStyleSheet("QPushButton {background-color: #21dbad;}")
                self.current_signal_button = button  # 현재 선택된 버튼 저장
                break

        self.check_timer()
        self.check_viz_timer()
    
    def click_selfdriving(self, value):
        self.RM.user_input[0] = value
        # 버튼 상태에 따른 배경색 설정
        if value == 0:  # stopButton 선택
            self.ui.stopButton.setStyleSheet("QPushButton {background-color: red;}")
            self.ui.startButton.setStyleSheet("")  # 기본 스타일로 리셋
        elif value == 1:  # startButton 선택
            self.ui.startButton.setStyleSheet("QPushButton {background-color: blue;}")
            self.ui.stopButton.setStyleSheet("")  # 기본 스타일로 리셋
            
        self.check_timer()
    
    def click_set(self):
        self.RM.user_input[2] = float(self.ui.velocityBox.value()/3.6)
        # 체크된 checkbox의 value를 찾고, 해당하는 radio button을 선택
        selected_value = None
        for checkbox, value in self.check_map.items():
            if checkbox.isChecked():
                self.RM.user_input[3] = value
                selected_value = value
                break
        
        # 해당하는 radio button을 선택 (체크박스와 라디오 버튼의 번호가 동일하다고 가정)
        if selected_value is not None:           
            self.click_load()
        self.check_timer()

    def check_timer(self):
        if not self.sig_in:
            self.sig_in = True
            self.user_input_timer.start(200)
            QTimer.singleShot(4000, self.stop_user_input_timer)
        else:
            self.stop_user_input_timer()
    
    def check_viz_timer(self):
        if not self.sig_in_viz:
            self.sig_in_viz = True
            self.user_input_viz_timer.start(200)
            QTimer.singleShot(4000, self.stop_user_input_viz_timer)
    
    def stop_user_input_timer(self):
        self.sig_in = False
        self.RM.user_input[1] = 0

        # signal button 스타일 리셋
        if hasattr(self, 'current_signal_button') and self.current_signal_button:
            self.current_signal_button.setStyleSheet("")
            self.current_signal_button = None

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
    
    def on_checkbox_changed(self, checkbox_value):
        """체크박스 상태가 변경될 때 해당하는 라디오 버튼을 선택"""
        # 해당하는 라디오 버튼을 선택하고 나머지는 해제
        for radio, radio_value in self.radio_point.items():
            if radio_value == checkbox_value:
                radio.setChecked(True)
            else:
                radio.setChecked(False)

    def state_triggered_viz(self):
        if self.ego_signal in [1,2]:
            self.ui.egoLabel.setStyleSheet("QLabel {background-color: #21dbad;}")
        if self.target_signal in [1,2]:
            self.ui.targetLabel.setStyleSheet("QLabel {background-color: #21dbad;}")
        if self.ego_signal == 4:
            self.ui.egoLabel.setStyleSheet("QLabel {background-color: #5471ff;}")
        if self.target_signal == 4:
            self.ui.targetLabel.setStyleSheet("QLabel {background-color: #5471ff;}")
        if self.ego_signal in [5,7]:
            self.ui.egoLabel.setStyleSheet("QLabel {background-color: #ff546b;}")
        if self.target_signal in [5,7]:
            self.ui.targetLabel.setStyleSheet("QLabel {background-color: #ff546b;}")

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
        for checkbox, value in self.check_map.items():
            checkbox.stateChanged.connect(partial(self.on_checkbox_changed, int(value)))
    
    def closeEvent(self, event):
        """애플리케이션 종료 시 리소스 정리"""
        self.RM.cleanup()
        event.accept()
    

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