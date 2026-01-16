import os
import time
import sys
import logging
from collections.abc import Iterable
import math
import subprocess
import shlex
import signal
from loguru import logger
import yaml
from datetime import datetime

import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as SciRotation

from PyQt5.QtWidgets import QApplication, QSlider, QPushButton, QMainWindow, QApplication, QFileDialog, QTextEdit, QWidget, QVBoxLayout, QLabel, QHBoxLayout, QGroupBox, QComboBox, QFrame, QGridLayout, QProgressBar, QFileDialog
from PyQt5.QtGui import QIcon, QPixmap, QMouseEvent, QFont, QVector3D, QImage, QTextCursor, QPainter, QColor, QPen, QPolygon, QPainterPath
from PyQt5.QtCore import qDebug, QTimer, QObject, QEvent, Qt, pyqtSignal, QThread, QSize, QPoint, QRectF

import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph import Transform3D, ColorMap, mkColor, makeRGBA

import rospy
from std_msgs.msg import String, Int8
from sensor_msgs.msg import JointState, Imu, Image as Image_msg, PointCloud2
from ros_numpy.image import image_to_numpy, numpy_to_image
from ros_numpy.point_cloud2 import pointcloud2_to_array
from geometry_msgs.msg import Twist, Vector3
from rosgraph_msgs.msg import Log

from dashboard import CompassWidget, ArtificialHorizonWidget
from thruster import RobotHUDWidget

# --- LOGO图片 ---
LOGO_IMAGE_PATH = "/home/caotl21/work/PYQT/qt_ros/my_example/logo/test_transparent_pil.png"  # 请确保图片在当前目录下

# --- 配色方案 (参考您的UI图) ---
COLOR_BG = QColor(30, 30, 35)           # 仪表盘深色背景
COLOR_RING = QColor(60, 60, 70)         # 仪表盘外圈
COLOR_TEXT = QColor(220, 220, 220)      # 白色文字/刻度
COLOR_SKY = QColor(60, 120, 180)        # 天空蓝
COLOR_GROUND = QColor(160, 100, 50)     # 地面棕橙色
COLOR_ACCENT_ORANGE = QColor(255, 140, 0) # 亮橙色 (飞机标志)
COLOR_ACCENT_RED = QColor(220, 50, 50)    # 红色 (罗盘指针)
COLOR_ACCENT_BLUE = QColor(0, 180, 255)   # 亮蓝色 (Roll指示)
#白色

# ===========================
# 样式与主题设置
# ===========================
DARK_STYLESHEET = """
QMainWindow {
    background-color: #2b2b2b;
}
QLabel {
    color: #d3d3d3;
    font-size: 12px;
}
QGroupBox {
    border: 1px solid #444444;
    border-radius: 5px;
    margin-top: 10px;
    color: #ffffff;
    font-weight: bold;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 3px 0 3px;
}
QPushButton {
    background-color: #3c3f41;
    color: #ffffff;
    border: 1px solid #555555;
    padding: 5px;
    border-radius: 3px;
}
QPushButton:hover {
    background-color: #484b4d;
}
QPushButton:pressed {
    background-color: #2b2b2b;
}
QProgressBar {
    border: 1px solid #555555;
    border-radius: 3px;
    text-align: center;
    color: white;
}
QProgressBar::chunk {
    background-color: #007acc; 
}
QTextEdit {
    background-color: #000000;
    color: #000000;
    border: 1px solid #444444;
}
"""

LIGHT_STYLESHEET = """
QMainWindow {
    background-color: #F5F7FA;
}
QLabel {
    color: #000000;
    font-size: 12px;
}
QGroupBox {
    border: 1px solid #AAAAAA;
    border-radius: 5px;
    margin-top: 15px;
    color: #000000;
    font-weight: bold;
    font-size: 14px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 15px;
    padding: 0 5px 0 5px;
    background-color: #f0f0f0;
    color: #000000;
    font-weight: bold;
}

QProgressBar {
    border: 1px solid #aaaaaa;
    border-radius: 3px;
    text-align: center;
    color: #000000;
}
QProgressBar::chunk {
    background-color: #007acc;
}
QTextEdit {
    background-color: #f0f0f0;
    color: #000000;
    border: 1px solid #cccccc;
    font-family: monospace;
    font-size: 10pt;
    border-radius: 8px;
}
QComboBox {
    background-color: #ffffff;
    color: #000000;
    border: 1px solid #aaaaaa;
    padding: 3px;
}
/* 框架样式 */

"""

class RosNode(QThread):
    data_arrive_signal = pyqtSignal()
    image_arrive_signal = pyqtSignal()
    connection_status_signal = pyqtSignal(bool, str)
    log_arrive_signal = pyqtSignal(str)
    imu_arrive_signal = pyqtSignal()
    thruster_arrive_signal = pyqtSignal()

    def __init__(self):
        super(RosNode, self).__init__()
        rospy.init_node('qt_ros_node')

        # 获取Master对象
        self.master = rospy.get_master()

        self.stop_flag = False
        self.loop_rate = rospy.Rate(30, reset=True)
        self.robot_namespace = ''  # 默认机器人命名空间
        
        self.simulation_publisher = None
        self.simulation_subscriber = None
        self.image_subscriber = None
        self.imu_subscriber = None
        self.thruster_subscriber = None
        
        self.msg = None
        self.image = None
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.is_connected = False # 连接状态标志

        rospy.Subscriber('/rosout', Log, self.rosout_callback, queue_size=100)
    
    def connect_robot(self, robot_namespace):
        """连接到指定的机器人"""

        #检查机器人话题是否存在
        excepted_topic = f'/{robot_namespace}/joint_states'

        # 获取系统状态
        self.rossystem_state = self.master.getSystemState()
        self.state = self.rossystem_state[2]  # 获取发布和订阅的话题列表
        self.pub_topics = self.state[0]
        self.sub_topics = self.state[1]

        self.all_ropics = set()
        for topic, _ in self.pub_topics:
            self.all_ropics.add(topic)
        for topic, _ in self.sub_topics:
            self.all_ropics.add(topic)
        topic_names = list(self.all_ropics)

        if excepted_topic not in topic_names:
            # 将下述日志同步发布到ROS日志系统中
            rospy.logerr(f"连接失败：{robot_namespace}")
            logger.error(f"连接失败：{robot_namespace}")
            self.connection_status_signal.emit(False, f"连接失败：{robot_namespace}")
            self.is_connected = False
            return
        
        self.robot_namespace = robot_namespace
        
        # 取消之前的订阅
        if self.simulation_subscriber:
            self.simulation_subscriber.unregister()
        if self.image_subscriber:
            self.image_subscriber.unregister()
        if self.imu_subscriber:
            self.imu_subscriber.unregister()
        if self.thruster_subscriber:
            self.thruster_subscriber.unregister()
        
        # 创建新的发布者和订阅者
        self.simulation_publisher = rospy.Publisher(f'/{robot_namespace}/cmd_vel', Twist, queue_size=1)
        self.simulation_subscriber = rospy.Subscriber(f'/{robot_namespace}/cmd_vel', Twist, callback=self.callback, queue_size=1)
        self.image_subscriber = rospy.Subscriber(f'/{robot_namespace}/camera/image_raw', Image_msg, callback=self.image_callback, queue_size=1)
        self.imu_subscriber = rospy.Subscriber(f'/{robot_namespace}/imu/data', Imu, callback=self.imu_callback, queue_size=1)
        self.thruster_subscriber = rospy.Subscriber(f'/{robot_namespace}/thruster_states', JointState, callback=self.thruster_callback, queue_size=1)
        
        # 将下述日志同步发布到ROS日志系统中
        rospy.loginfo(f"连接成功：{robot_namespace}")
        logger.info(f"连接成功：{robot_namespace}")
        self.connection_status_signal.emit(True, f"连接成功：{robot_namespace}")
        
        self.is_connected = True

    def rosout_callback(self, msg):
        if msg.level < 2:
            return
        
        level_dict = {1: 'DEBUG', 2: 'INFO', 4: 'WARN', 8: 'ERROR', 16: 'FATAL'}
        level_name = level_dict.get(msg.level, 'UNKNOWN')
        #这里使用的是ROS的Gazebo仿真时间
        #msg_time = datetime.fromtimestamp(msg.header.stamp.to_sec()).strftime('%Y-%m-%d %H:%M:%S')
        msg_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        if msg.name.startswith('/rosout') or msg.name.startswith('/gazebo'):
            return
        formatted_msg = f"[{level_name}][{msg_time}][{msg.name}]: {msg.msg}\n"
        self.log_arrive_signal.emit(formatted_msg)
    
    def thruster_callback(self, msg):
        thrust_values = []
        for i in range(6):
            try:
                index = msg.name.index(f'thruster_{i+1}_joint')
                thrust = msg.effort[index]
                thrust_values.append(thrust)
            except ValueError:
                thrust_values.append(0.0)
        
        self.thruster_values = thrust_values
        self.thruster_arrive_signal.emit()
    
    def imu_callback(self, msg):
        # 提取欧拉角（弧度）
        orientation_q = msg.orientation
        r = SciRotation.from_quat([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.roll, self.pitch, self.yaw = r.as_euler('xyz', degrees=True)
        rospy.loginfo(f"IMU数据到达: Roll={self.roll:.2f}, Pitch={self.pitch:.2f}, Yaw={self.yaw:.2f}")
        self.imu_arrive_signal.emit()

    def image_callback(self, msg):
        self.image = image_to_numpy(msg)
        self.image_arrive_signal.emit()

    def callback(self, msg):
        self.msg = msg
        self.data_arrive_signal.emit()

    def publish_twist_message(self):
        if not self.is_connected:
            logger.warning("请先连接机器人!")
            return
            
        sender = self.sender()
        message = Twist()
        if sender.text() == 'FORWARD':
            message.linear = Vector3(1, 0, 0)
            message.angular = Vector3(0, 0, 0)
        elif sender.text() == 'BACK':
            message.linear = Vector3(-1, 0, 0)
            message.angular = Vector3(0, 0, 0)
        elif sender.text() == 'LEFT':
            message.linear = Vector3(0, 0, 0)
            message.angular = Vector3(0, 0, 1)
        elif sender.text() == 'RIGHT':
            message.linear = Vector3(0, 0, 0)
            message.angular = Vector3(0, 0, -1)
        elif sender.text() == 'WAIT':
            message.linear = Vector3(0, 0, 0)
            message.angular = Vector3(0, 0, 0)
        self.simulation_publisher.publish(message)
        

    def run(self) -> None:
        while not rospy.is_shutdown() and not self.stop_flag:
            self.loop_rate.sleep()

class RosLogHandler(logging.Handler):
    def __init__(self, log_signal):
        super().__init__()
        # 接收并保存一个信号，用于在UI线程中传递日志消息
        self.log_signal = log_signal
    
    def emit(self, record):
        try:
            # 将日志对象转化为字符串消息：应用设置的formatter
            msg = self.format(record)
            formatted_msg = f"[{record.levelname}] {msg}"
            self.log_signal.emit(formatted_msg)
        except Exception:
            pass

# 重定向标准输出流的类
class StreamToQt(object):
    def __init__(self, signal):
        self.signal = signal

    def write(self, text):
        # 实时捕获终端输出，并通过信号发出去
        # 使用 str() 确保是字符串
        self.signal.emit(str(text))

    def flush(self):
        # 必须实现这个方法，因为 python 打印时会调用它
        pass

class Widget(QMainWindow):

    # 添加一个信号用于在UI线程中记录日志
    #log_signal = pyqtSignal(str)

    def __init__(self):
        super(Widget, self).__init__()
        self.setMinimumSize(1600, 950)
        self.init_ui()
        self.setWindowTitle("BRICS 水下机器人地面站 v2.0")
        self.setStyleSheet(LIGHT_STYLESHEET)

        self.ros_node = RosNode()
        self.ros_node.start()

    def init_ui(self):
        main_widget = QWidget()
        main_layout = QVBoxLayout()
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

        # 1. 顶部栏
        top_bar = self.create_top_bar()
        main_layout.addWidget(top_bar, 5)  # 占5%高度

        # 2. 中间区域 (左侧数据 + 摄像头 + 右侧数据)
        center_layout = QHBoxLayout()
        
        left_panel = self.create_left_panel()
        camera_panel = self.create_camera_panel()
        right_panel = self.create_right_panel()

        center_layout.addWidget(left_panel, 20)    # 占20%宽度
        center_layout.addWidget(camera_panel, 60)  # 占60%宽度
        center_layout.addWidget(right_panel, 20)   # 占20%宽度
        main_layout.addLayout(center_layout, 90)   # 占85%高度

        # 3. 底部栏
        bottom_bar = self.create_bottom_bar()
        main_layout.addWidget(bottom_bar, 5)      # 占10%高度

    # =======================================================
    # 顶部状态栏：选择机器人，连接状态，运动模式选择，固件更新
    # =======================================================
    def create_top_bar(self):
        
        top_bar = QFrame()
        top_bar.setFrameShape(QFrame.StyledPanel)
        top_layout = QHBoxLayout()
        top_bar.setLayout(top_layout)

        ################# 添加组件到顶部栏 ####################
        select_robot_box = QGroupBox()
        #select_robot_box.setMaximumHeight(120)
        #select_robot_box.setMaximumWidth(300)
        select_robot_layout = QGridLayout()
        select_robot_box.setLayout(select_robot_layout)
        self.select_robot = QLabel('选择机器人:')
        select_robot_layout.addWidget(self.select_robot,0, 0)

        # 添加机器人选择下拉框
        self.robot_combo = QComboBox()
        self.robot_combo.addItems(['robot1', 'robot2'])
        self.robot_combo.setMaximumHeight(30)
        select_robot_layout.addWidget(self.robot_combo, 0, 1)

        # 添加连接按钮
        self.connect_button = QPushButton('连接机器人')
        self.connect_button.setMinimumHeight(30)
        self.connect_button.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        select_robot_layout.addWidget(self.connect_button, 0, 2)

        # 添加状态标签
        self.status_label = QLabel('等待连接中...')
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("background-color: #ff6666; color: white; font-weight: bold; padding: 5px;")
        self.status_label.setMaximumHeight(30)
        select_robot_layout.addWidget(self.status_label, 1, 0, 1, 3)

        # 添加显示机器人ip和连接延迟的组件 上层是ip_label 下层是latency_label
        self.ip_label = QLabel('IP地址:')
        self.ip_label.setAlignment(Qt.AlignCenter)
        #self.ip_label.setStyleSheet("background-color: #f0f0f0; padding: 8px;")
        self.ip_label.setMinimumHeight(10)
        select_robot_layout.addWidget(self.ip_label, 0, 3)
        self.ip_value = QLabel('192.168.1.x')
        self.ip_value.setAlignment(Qt.AlignCenter)
        #self.ip_value.setStyleSheet("border: 1px solid gray; background-color: #f0f0f0; padding: 5px;")
        self.ip_value.setMinimumHeight(10)
        select_robot_layout.addWidget(self.ip_value, 0, 4)
        self.latency_label = QLabel('通信延迟:')
        self.latency_label.setAlignment(Qt.AlignCenter)
        #self.latency_label.setStyleSheet("border: 1px solid gray; background-color: #f0f0f0; padding: 5px;")
        self.latency_label.setMinimumHeight(10)
        select_robot_layout.addWidget(self.latency_label, 1, 3)
        self.latency_value = QLabel('ping unknown')
        self.latency_value.setAlignment(Qt.AlignCenter)
        #self.latency_value.setStyleSheet("border: 1px solid gray; background-color: #f0f0f0; padding: 5px;")
        self.latency_value.setMinimumHeight(10)
        select_robot_layout.addWidget(self.latency_value, 1, 4)
        self.connection_time_label = QLabel('连接时间:')
        self.connection_time_label.setAlignment(Qt.AlignCenter) 
        #self.connection_time_label.setStyleSheet("border: 1px solid gray; background-color: #f0f0f0; padding: 5px;")
        self.connection_time_label.setMinimumHeight(10)
        select_robot_layout.addWidget(self.connection_time_label, 0, 5)
        self.connection_time_value = QLabel('00:00:00')
        self.connection_time_value.setAlignment(Qt.AlignCenter)
        #self.connection_time_value.setStyleSheet("border: 1px solid gray; background-color: #f0f0f0; padding: 5px;")
        self.connection_time_value.setMinimumHeight(10)
        select_robot_layout.addWidget(self.connection_time_value, 0, 6)
        self.stm32_label = QLabel('主控状态:')
        self.stm32_label.setAlignment(Qt.AlignCenter)
        #self.stm32_label.setStyleSheet("border: 1px solid gray; background-color: #f0f0f0; padding: 5px;")
        self.stm32_label.setMinimumHeight(10)
        select_robot_layout.addWidget(self.stm32_label, 1, 5)
        self.stm32_value = QLabel('F407 Unknown')
        self.stm32_value.setAlignment(Qt.AlignCenter)
        #self.stm32_value.setStyleSheet("border: 1px solid gray; background-color: #f0f0f0; padding: 5px;")
        self.stm32_value.setMinimumHeight(10)
        select_robot_layout.addWidget(self.stm32_value, 1, 6)

        top_layout.addWidget(select_robot_box)

        # 添加运动模式选择组件
        select_mode_box = QGroupBox()
        select_mode_box.setMaximumHeight(120)
        select_mode_box.setMaximumWidth(300)
        select_mode_layout = QGridLayout()
        select_mode_box.setLayout(select_mode_layout)

        mode_label = QLabel('选择运动模式:')
        select_mode_layout.addWidget(mode_label, 0, 0)
        
        # 添加运动模式选择下拉框
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(['手动模式', '自动定深'])
        self.mode_combo.setMaximumHeight(30)
        select_mode_layout.addWidget(self.mode_combo, 0, 1)

        # 添加确认选择按钮
        self.mode_button = QPushButton('确认模式')
        self.mode_button.setMinimumHeight(30)
        # 蓝色
        self.mode_button.setStyleSheet("background-color: #007BFF; color: white; font-weight: bold;")
        select_mode_layout.addWidget(self.mode_button, 0, 2)
        # 添加选择状态标签
        self.mode_status_label = QLabel('未进入控制模式...')
        self.mode_status_label.setAlignment(Qt.AlignCenter)
        self.mode_status_label.setMaximumHeight(30)
        # 淡红色背景 字体白色
        self.mode_status_label.setStyleSheet("background-color: #ff6666; color: white; font-weight: bold;padding: 5px;")
        select_mode_layout.addWidget(self.mode_status_label, 1, 0, 1, 3)

        top_layout.addWidget(select_mode_box)

        # 添加固件更新选择组件
        OTA_box = QGroupBox()
        OTA_box.setMaximumHeight(120)
        OTA_box.setMaximumWidth(300)
        OTA_layout = QGridLayout()
        OTA_box.setLayout(OTA_layout)
        OTA_label = QLabel('选择固件文件:')
        OTA_layout.addWidget(OTA_label, 0, 0)
        
        # 添加更新固件选择下拉框
        self.ota_combo = QComboBox()
        self.ota_combo.addItems(['手动模式', '自动定深'])
        self.ota_combo.setMaximumHeight(30)
        OTA_layout.addWidget(self.ota_combo, 0, 1)

        # 添加确认选择按钮
        self.ota_button = QPushButton('开始更新')
        self.ota_button.setMinimumHeight(30)
        # 卡其色
        self.ota_button.setStyleSheet("background-color: #F0E68C; color: white; font-weight: bold;")
        OTA_layout.addWidget(self.ota_button, 0, 2)
        # 添加选择状态标签
        self.ota_status_label = QLabel('未进入更新模式...')
        self.ota_status_label.setAlignment(Qt.AlignCenter)
        self.ota_status_label.setMaximumHeight(30)
        # 淡红色背景 字体白色
        self.ota_status_label.setStyleSheet("background-color: #ff6666; color: white; font-weight: bold;padding: 5px;")
        OTA_layout.addWidget(self.ota_status_label, 1, 0, 1, 3)

        top_layout.addWidget(OTA_box)



        top_layout.addStretch(1) # 弹簧占位

        # 添加logo图片
        logo_label = QLabel()
        logo_pixmap = QPixmap(LOGO_IMAGE_PATH)  # 请确保logo.png在当前目录下
        logo_pixmap = logo_pixmap.scaledToHeight(90, Qt.SmoothTransformation)
        logo_label.setPixmap(logo_pixmap)
        top_layout.addWidget(logo_label)

        return top_bar
    
    def create_left_panel(self):
        left_panel = QFrame()
        left_panel.setFrameShape(QFrame.StyledPanel)
        left_layout = QVBoxLayout()
        left_panel.setLayout(left_layout)

        # 仪表盘区域
        dashboard_layout = QHBoxLayout()
        self.compass_widget = CompassWidget()
        self.horizon_widget = ArtificialHorizonWidget()
        left_dashboard_layout = QVBoxLayout()
        left_dashboard_layout.addWidget(QLabel('电子罗盘:'))
        left_dashboard_layout.addWidget(self.compass_widget)
        right_dashboard_layout = QVBoxLayout()
        right_dashboard_layout.addWidget(QLabel('姿态仪表:'))
        right_dashboard_layout.addWidget(self.horizon_widget)
        dashboard_layout.addLayout(left_dashboard_layout)
        dashboard_layout.addLayout(right_dashboard_layout)
        left_layout.addLayout(dashboard_layout)

        # 双IMU数值显示区域
        gb_imu = QGroupBox("双IMU数据 (IMU Data Comparison)")
        imu_layout = QGridLayout()
        imu_layout.addWidget(QLabel(""), 0, 0)
        imu_layout.addWidget(QLabel("IMU1"), 0, 1)
        imu_layout.addWidget(QLabel("IMU2"), 0, 2)
        imu_layout.addWidget(QLabel("误差"), 0, 3)

        self.lbl_imu1_r = QLabel("0.0°")
        self.lbl_imu1_p = QLabel("0.0°")
        self.lbl_imu1_y = QLabel("0.0°")
        self.lbl_imu2_r = QLabel("0.0°")
        self.lbl_imu2_p = QLabel("0.0°")
        self.lbl_imu2_y = QLabel("0.0°")
        self.lbl_imu_error_r = QLabel("0.0°")
        self.lbl_imu_error_p = QLabel("0.0°")
        self.lbl_imu_error_y = QLabel("0.0°")

        imu_layout.addWidget(QLabel("Roll (横滚):"), 1, 0)
        imu_layout.addWidget(self.lbl_imu1_r, 1, 1)
        imu_layout.addWidget(self.lbl_imu2_r, 1, 2)
        imu_layout.addWidget(self.lbl_imu_error_r, 1, 3)
        imu_layout.addWidget(QLabel("Pitch (俯仰):"), 2, 0)
        imu_layout.addWidget(self.lbl_imu1_p, 2, 1)
        imu_layout.addWidget(self.lbl_imu2_p, 2, 2)
        imu_layout.addWidget(self.lbl_imu_error_p, 2, 3)
        imu_layout.addWidget(QLabel("Yaw (航向):"), 3, 0)
        imu_layout.addWidget(self.lbl_imu1_y, 3, 1)
        imu_layout.addWidget(self.lbl_imu2_y, 3, 2)
        imu_layout.addWidget(self.lbl_imu_error_y, 3, 3)

        gb_imu.setLayout(imu_layout)
        left_layout.addWidget(gb_imu)

        # 机器人运动状态：速度 加速度 角速度 角加速度 一共两排 第一排是标签 第二排是具体值
        self.lbl_velocity = QLabel("0.00")
        self.lbl_acceleration = QLabel("0.00")
        self.lbl_angular_velocity = QLabel("0.00")
        self.lbl_angular_acceleration = QLabel("0.00")

        gb_motion = QGroupBox("机器人运动状态 (Robot Motion State)")
        motion_layout = QGridLayout()
        motion_layout.addWidget(QLabel("线速度(m/s)"), 0, 0)
        motion_layout.addWidget(QLabel("加速度(m/s²)"), 0, 1)
        motion_layout.addWidget(QLabel("角速度(°/s)"), 0, 2)
        motion_layout.addWidget(QLabel("角加速度(°/s²)"), 0, 3)

        motion_layout.addWidget(self.lbl_velocity, 1, 0)
        motion_layout.addWidget(self.lbl_acceleration, 1, 1)
        motion_layout.addWidget(self.lbl_angular_velocity, 1, 2)
        motion_layout.addWidget(self.lbl_angular_acceleration, 1, 3)

        gb_motion.setLayout(motion_layout)
        left_layout.addWidget(gb_motion)

        # 环境检测：外部水温 内部水温
        gb_env = QGroupBox("状态环境参数 (Environmental Parameters)")
        env_layout = QGridLayout()
        self.lbl_temp_water_pressure = QLabel("0.0 kPa")
        self.lbl_temp_water = QLabel("0.0 °C")
        self.lbl_temp_internal = QLabel("0.0 °C")
        self.lbl_humi_internal = QLabel("0 %")
        
        env_layout.addWidget(QLabel("外部水压(kPa)"), 0, 0)
        env_layout.addWidget(self.lbl_temp_water_pressure, 1, 0)
        env_layout.addWidget(QLabel("外部水温(°C)"), 0, 1)
        env_layout.addWidget(self.lbl_temp_water, 1, 1)
        env_layout.addWidget(QLabel("舱内温度(°C)"), 0, 2)
        env_layout.addWidget(self.lbl_temp_internal, 1, 2)
        env_layout.addWidget(QLabel("舱内湿度(%)"), 0, 3)
        env_layout.addWidget(self.lbl_humi_internal, 1, 3)
        gb_env.setLayout(env_layout)
        left_layout.addWidget(gb_env)

        # 深度计:显示形式是 xx | xx 分别表示实际值和设定值 
        gb_depth = QGroupBox("深度计 (Depth Sensor)")
        depth_layout = QVBoxLayout()

        # 创建水平布局显示实际值和设定值
        depth_values_layout = QHBoxLayout()

        # 实际深度标签
        self.lbl_depth_actual = QLabel("0.00")
        self.lbl_depth_actual.setStyleSheet("font-size: 32px; color: #00008b; font-weight: bold;")
        self.lbl_depth_actual.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        # 分隔符 改为黑色粗体竖线
        separator_label = QLabel("|")
        separator_label.setStyleSheet("font-size: 32px; color: #000000; font-weight: bold;")
        separator_label.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)

        # 设定深度标签
        self.lbl_depth_setpoint = QLabel("0.00")
        self.lbl_depth_setpoint.setStyleSheet("font-size: 32px; color: #4682b4; font-weight: bold;")
        self.lbl_depth_setpoint.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

        # 添加到水平布局
        depth_values_layout.addWidget(self.lbl_depth_actual)
        depth_values_layout.addWidget(separator_label)
        depth_values_layout.addWidget(self.lbl_depth_setpoint)

        # 组装深度计布局
        depth_layout.addLayout(depth_values_layout)
        gb_depth.setLayout(depth_layout)
        left_layout.addWidget(gb_depth)

        return left_panel
        
    def create_camera_panel(self):
        camera_panel = QFrame()
        camera_panel.setFrameShape(QFrame.StyledPanel)
        camera_layout = QVBoxLayout()
        camera_panel.setLayout(camera_layout)

        # 图片标签
        self.image_label = QLabel('Video Flow Waiting...')
        self.image_label.setMinimumSize(600, 400)
        self.image_label.setStyleSheet("border: 1px solid black;")
        self.image_label.setAlignment(Qt.AlignCenter)
        camera_layout.addWidget(self.image_label)

        # 画一个透明蒙板 在image_label上面 显示中心瞄准标志
        '''self.overlay_label = QLabel(self.image_label)
        self.overlay_label.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.overlay_label.setStyleSheet("background-color: rgba(0, 0, 0, 0);")
        self.overlay_label.setGeometry(0, 0, self.image_label.width(), self.image_label.height())   
        camera_layout.addWidget(self.overlay_label)'''

        return camera_panel
    
    def create_right_panel(self):
        right_panel = QFrame()
        right_panel.setFrameShape(QFrame.StyledPanel)
        right_layout = QVBoxLayout()
        right_panel.setLayout(right_layout)

        # 添加推进器显示组件
        gb_thruster = QGroupBox("推进器状态 (Thruster Status)")
        self.thruster_widget = RobotHUDWidget()
        gb_thruster.setLayout(QVBoxLayout())
        gb_thruster.layout().addWidget(self.thruster_widget)
        right_layout.addWidget(gb_thruster)

        # 显示二维云台舵机角度：条形式显示
        gb_gimbal = QGroupBox("云台舵机角度 (Gimbal Angles)")
        gimbal_layout = QGridLayout()
        self.angle_1_slider = QSlider(Qt.Horizontal)
        self.angle_1_slider.setMinimum(-90)
        self.angle_1_slider.setMaximum(90)
        self.angle_1_slider.setValue(0)
        self.angle_1_slider.setEnabled(True)
        self.angle_1_slider.setTickPosition(QSlider.TicksBelow)  # 显示刻度
        self.angle_1_slider.setTickInterval(10)  #刻度间隔

        self.angle_2_slider = QSlider(Qt.Horizontal)
        self.angle_2_slider.setMinimum(0)
        self.angle_2_slider.setMaximum(180)
        self.angle_2_slider.setValue(90)
        self.angle_2_slider.setEnabled(True)
        self.angle_2_slider.setTickPosition(QSlider.TicksBelow)  # 显示刻度
        self.angle_2_slider.setTickInterval(10)  #刻度间隔
        gimbal_layout.addWidget(QLabel("水平旋转角度:"), 0, 0)
        gimbal_layout.addWidget(self.angle_1_slider, 0, 1)
        gimbal_layout.addWidget(QLabel("垂直俯仰角度:"), 1, 0)
        gimbal_layout.addWidget(self.angle_2_slider, 1, 1)
        gb_gimbal.setLayout(gimbal_layout)
        right_layout.addWidget(gb_gimbal)   


        # 显示XYZ三个方向上的推力及回转力矩的设定值
        gb_thrust = QGroupBox("推力与力矩设定值 (Thrust and Torque)")
        thrust_layout = QGridLayout()
        self.lbl_thrust_x = QLabel("0.0 N")
        self.lbl_thrust_y = QLabel("0.0 N")
        self.lbl_thrust_z = QLabel("0.0 N")
        self.lbl_torque_x = QLabel("0.0 Nm")
        self.lbl_torque_y = QLabel("0.0 Nm")
        self.lbl_torque_z = QLabel("0.0 Nm")
        thrust_layout.addWidget(QLabel(""), 0, 0)
        thrust_layout.addWidget(QLabel("X方向"), 0, 1)
        thrust_layout.addWidget(QLabel("Y方向"), 0, 2)
        thrust_layout.addWidget(QLabel("Z方向"), 0, 3)
        thrust_layout.addWidget(QLabel("推力 (N):"), 1, 0)
        thrust_layout.addWidget(self.lbl_thrust_x, 1, 1)
        thrust_layout.addWidget(self.lbl_thrust_y, 1, 2)
        thrust_layout.addWidget(self.lbl_thrust_z, 1, 3)
        thrust_layout.addWidget(QLabel("力矩 (Nm):"), 2, 0)
        thrust_layout.addWidget(self.lbl_torque_x, 2, 1)
        thrust_layout.addWidget(self.lbl_torque_y, 2, 2)
        thrust_layout.addWidget(self.lbl_torque_z, 2, 3)
        gb_thrust.setLayout(thrust_layout)
        right_layout.addWidget(gb_thrust)

        # 电池状态：电压 电流 剩余容量 分布情况是：电压电流是一行 剩余容量是一行且剩余容量使用一个进度条显示
        gb_battery = QGroupBox("电池状态 (Battery Status)")
        battery_layout = QGridLayout()
        self.lbl_voltage = QLabel("0.0 V")
        self.lbl_current = QLabel("0.0 A")
        self.battery_progress = QProgressBar()
        self.battery_progress.setValue(0)
        self.battery_progress.setStyleSheet("background-color: #d0d0d0;")
        self.battery_progress.setMinimumHeight(20)

        battery_layout.addWidget(QLabel("电压 (Voltage):"), 0, 0)
        battery_layout.addWidget(self.lbl_voltage, 0, 1)
        battery_layout.addWidget(QLabel("电流 (Current):"), 0, 2)
        battery_layout.addWidget(self.lbl_current, 0, 3)
        battery_layout.addWidget(QLabel("剩余容量 (Capacity):"), 1, 0)
        battery_layout.addWidget(self.battery_progress, 1, 1, 1, 3)
        gb_battery.setLayout(battery_layout)
        right_layout.addWidget(gb_battery)

        return right_panel
        
    def create_bottom_bar(self):
        bottom_bar = QFrame()
        bottom_bar.setFrameShape(QFrame.StyledPanel)
        bottom_layout = QHBoxLayout()
        bottom_bar.setLayout(bottom_layout)

        bottom_button_layout = QVBoxLayout()
        bottom_layout.addLayout(bottom_button_layout)

        # 开始拍照按钮
        self.btn_take_photo = QPushButton("即刻拍照")
        self.btn_take_photo.setStyleSheet("background-color: #5cb85c; color: white; font-weight: bold; font-size: 16px; Width: 120px; Height: 35px;")
        bottom_button_layout.addWidget(self.btn_take_photo)

        # 开始录像按钮
        self.btn_start_recording = QPushButton("开始录像")
        self.btn_start_recording.setStyleSheet("background-color: #5bc0de; color: white; font-weight: bold; font-size: 16px; Width: 120px; Height: 35px;")
        bottom_button_layout.addWidget(self.btn_start_recording)
        
        # 紧急停止按钮 
        self.btn_estop = QPushButton("紧急停止")
        self.btn_estop.setStyleSheet("background-color: #d9534f; color: white; font-weight: bold; font-size: 16px; Width: 120px; Height: 35px;")
        bottom_button_layout.addWidget(self.btn_estop)        

        # 日志文本编辑框
        self.log_text_edit = QTextEdit()
        self.log_text_edit.setReadOnly(True)
        self.log_text_edit.setMinimumHeight(150)
        self.log_text_edit.setStyleSheet("font-family: monospace; font-size: 10pt;")
        bottom_layout.addWidget(self.log_text_edit)

        return bottom_bar
        
       

    '''def init_ui(self):
        # 主布局：上中下
        main_layout = QVBoxLayout(self)
        # 顶部布局：横向布局
        top_layout = QHBoxLayout()
        # 中层布局：横向布局
        center_layout = QHBoxLayout()
        # 底部布局：横向布局
        bottom_layout = QHBoxLayout()

        dashboard_layout = QVBoxLayout()
        
        # 左侧垂直布局（按钮和文本）
        left_layout = QVBoxLayout()

        # 仪表盘区域
        self.compass_widget = CompassWidget()
        self.horizon_widget = ArtificialHorizonWidget()
        dashboard_layout.addWidget(QLabel('电子罗盘:'))
        dashboard_layout.addWidget(self.compass_widget)
        dashboard_layout.addWidget(QLabel('姿态仪表:'))
        dashboard_layout.addWidget(self.horizon_widget)
        
        # 右侧图片标签
        self.image_label = QLabel('Image Show')
        self.image_label.setMinimumSize(300, 200)
        self.image_label.setStyleSheet("border: 1px solid black; background-color: #f0f0f0;")
        self.image_label.setAlignment(Qt.AlignCenter)
        
        # 添加机器人选择下拉框
        self.robot_combo = QComboBox()
        self.robot_combo.addItems(['robot1', 'robot2'])
        self.robot_combo.setMinimumHeight(30)
        
        # 添加连接按钮
        self.connect_button = QPushButton('连接机器人')
        self.connect_button.setMinimumHeight(35)
        self.connect_button.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        
        # 添加状态标签
        self.status_label = QLabel('未连接')
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("border: 1px solid gray; background-color: #ffcccc; padding: 5px;")
        self.status_label.setMinimumHeight(30)

        # 创建按钮分别是：前进 后退 左转 右转 原地等待
        self.up_button = QPushButton('FORWARD')
        self.down_button = QPushButton('BACK')
        self.left_button = QPushButton('LEFT')
        self.right_button = QPushButton('RIGHT')
        self.wait_button = QPushButton('WAIT')
        
        # 将组件添加到左侧布局
        left_layout.addWidget(QLabel('选择机器人:'))
        left_layout.addWidget(self.robot_combo)
        left_layout.addWidget(self.connect_button)
        left_layout.addWidget(self.status_label)
        left_layout.addSpacing(20)  # 添加间距
        left_layout.addWidget(self.up_button)
        left_layout.addWidget(self.down_button)
        left_layout.addWidget(self.left_button)
        left_layout.addWidget(self.right_button)
        left_layout.addWidget(self.wait_button)
        left_layout.addStretch(1)
        
        # 将左右布局添加到顶部布局
        top_layout.addLayout(left_layout)
        top_layout.addWidget(self.image_label)
        top_layout.addLayout(dashboard_layout)

        # 添加推进器显示组件
        self.thruster_widget = RobotHUDWidget()
        top_layout.addWidget(self.thruster_widget)

        
        # 设置布局比例
        top_layout.setStretch(0, 1)
        top_layout.setStretch(1, 3)


        # 底部日志区域
        self.log_text_edit = QTextEdit()
        self.log_text_edit.setReadOnly(True)
        self.log_text_edit.setMinimumHeight(150)
        self.log_text_edit.setStyleSheet("background-color: #f0f0f0; font-family: monospace; font-size: 10pt;")

        # 将顶部布局和日志区域添加到主布局
        main_layout.addLayout(top_layout)
        main_layout.addWidget(self.log_text_edit)

        # QT界面Title设置
        self.setWindowTitle('水下机器人控制地面站')

        # 设置日志处理器
        #self.setup_logging()
        #sys.stdout = StreamToQt(self.log_signal)
        #sys.stderr = StreamToQt(self.log_signal)

        # 将ROS的节点传入QT的类中
        self.ros_node = RosNode()
        self.ros_node.start()

        # 连接信号
        self.connect_button.clicked.connect(self.connect_to_robot)
        self.ros_node.image_arrive_signal.connect(self.show_image)
        self.ros_node.connection_status_signal.connect(self.update_connection_status)
        self.ros_node.log_arrive_signal.connect(self.append_log)
        self.ros_node.imu_arrive_signal.connect(self.show_dashboard)
        self.ros_node.thruster_arrive_signal.connect(self.show_thrusters)

        
        self.up_button.clicked.connect(self.ros_node.publish_twist_message)
        self.down_button.clicked.connect(self.ros_node.publish_twist_message)
        self.right_button.clicked.connect(self.ros_node.publish_twist_message)
        self.left_button.clicked.connect(self.ros_node.publish_twist_message)
        self.wait_button.clicked.connect(self.ros_node.publish_twist_message)'''
        

    def connect_to_robot(self):
        """连接到选定的机器人"""
        robot_name = self.robot_combo.currentText()
        self.status_label.setText(f'正在连接到 {robot_name}...')
        self.status_label.setStyleSheet("border: 1px solid gray; background-color: #ffffcc; padding: 5px;")
        self.ros_node.connect_robot(robot_name)

    def update_connection_status(self, success, message):
        """更新连接状态标签"""
        if success:
            self.status_label.setText(message)
            self.status_label.setStyleSheet("border: 1px solid gray; background-color: #ccffcc; padding: 5px;")
        else:
            self.status_label.setText(message)
            self.status_label.setStyleSheet("border: 1px solid gray; background-color: #ffcccc; padding: 5px;")

    def mannul_servo_angle_changed(self, value):
        """当用户手动拖动滑块时触发"""
        #判断是哪个slider触发的
        sender = self.sender()
        if sender == self.angle_1_slider:
            self.angle_1_value_label.setText(f"{value} °")
            self.ros_node.publish_target_servo_angle(value, 1)
        elif sender == self.angle_2_slider:
            self.angle_2_value_label.setText(f"{value} °")
            self.ros_node.publish_target_servo_angle(value, 2)

    def show_image(self):
        image = self.ros_node.image
        show_image = QImage(image, image.shape[1], image.shape[0], image.shape[1]*3, QImage.Format_RGB888)
        self.image_label.setPixmap(QPixmap(show_image))

    def show_dashboard(self):
        self.compass_widget.set_heading(self.ros_node.yaw)
        self.horizon_widget.set_attitude(self.ros_node.roll, self.ros_node.pitch)

    def show_thrusters(self):
        self.thruster_widget.update_thrusts(self.ros_node.thruster_values)


    def setup_logging(self):
        ros_handler = RosLogHandler(self.log_signal)
        formatter = logging.Formatter('%(asctime)s-%(levelname)s-%(message)s')
        formatter.converter = time.localtime
        ros_handler.setFormatter(formatter)
        
        ros_logger = logging.getLogger()
        ros_logger.addHandler(ros_handler)
        ros_logger.setLevel(logging.INFO)

    def append_log(self, message):
        """
        槽函数：接收并显示日志
        """
        self.log_text_edit.moveCursor(QTextCursor.End)

        # 可以根据日志内容设置颜色
        if "[ERROR]" in message or "[FATAL]" in message:
            self.log_text_edit.setTextColor(Qt.red)
        elif "[WARN]" in message:
            self.log_text_edit.setTextColor(QColor(255, 165, 0)) # Orange
        else:
            self.log_text_edit.setTextColor(Qt.black)

        self.log_text_edit.insertPlainText(message)
        # 自动滚动到底部
        self.log_text_edit.ensureCursorVisible()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = Widget()
    w.show()
    sys.exit(app.exec_())
