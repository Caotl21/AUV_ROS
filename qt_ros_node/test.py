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
from std_msgs.msg import String, Int8, Float32MultiArray
from sensor_msgs.msg import JointState, Imu, Image as Image_msg, PointCloud2
from sensor_msgs.msg import FluidPressure as press
from ros_numpy.image import image_to_numpy, numpy_to_image
from ros_numpy.point_cloud2 import pointcloud2_to_array
from geometry_msgs.msg import Twist, Vector3
from rosgraph_msgs.msg import Log

from dashboard import CompassWidget, ArtificialHorizonWidget
from thruster import RobotHUDWidget

# --- 图片路径 ---
LOGO_IMAGE_PATH = "/home/caotl21/work/PYQT/qt_ros/my_example/logo/test_transparent_pil.png"  # 请确保图片在当前目录下
SCREENSHOT_PATH = "/home/caotl21/work/PYQT/qt_ros/my_example/screenshot/"  # 截图保存路径
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

# ==========================================
# 样式与主题设置
# ==========================================
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
    border-radius: 6px;
    margin-top: 6px;
    color: #000000;
    font-weight: bold;
    font-size: 14px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 15px;
    padding: 0 5px 0 5px;
    background-color: white;
    color: #2980B9;
    font-weight: bold;
}
/* 进度条样式 */
QProgressBar {
    border: 1px solid #BDC3C7;
    border-radius: 5px;
    text-align: center;
    color: #2C3E50;
    background-color: #ECF0F1;
    font-weight: bold;
}

QProgressBar::chunk {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #3498DB, stop:1 #2ECC71);
    border-radius: 4px;
}

QTextEdit {
    background-color: #f0f0f0;
    color: #000000;
    border: 1px solid #cccccc;
    font-family: monospace;
    font-size: 10pt;
    border-radius: 8px;
}

/* 下拉框样式 */
QComboBox {
    background-color: white;
    color: #2C3E50;
    border: 1px solid #BDC3C7;
    border-radius: 5px;
    padding: 3px;
    min-height: 20px;
    font-size: 13px;
}

QComboBox:hover {
    border: 1px solid #3498DB;
}

QComboBox::drop-down {
    border: none;
    width: 25px;
}

QComboBox::down-arrow {
    image: url(down_arrow.png);
    width: 12px;
    height: 12px;
}
/* 状态标签样式 */
.status-label {
    border: 2px solid #BDC3C7;
    border-radius: 6px;
    padding: 8px;
    font-weight: bold;
    font-size: 13px;
}

.status-success {
    background-color: #D5F4E6;
    color: #1E8449;
    border-color: #52BE80;
}

.status-warning {
    background-color: #FCF3CF;
    color: #9A7D0A;
    border-color: #F4D03F;
}

.status-error {
    background-color: #FADBD8;
    color: #943126;
    border-color: #EC7063;
}

.status-info {
    background-color: #D6EAF8;
    color: #1F618D;
    border-color: #5DADE2;
}
/* 滑块样式 */
QSlider::groove:horizontal {
    border: 1px solid #BDC3C7;
    height: 8px;
    background: #ECF0F1;
    margin: 2px 0;
    border-radius: 4px;
}

QSlider::handle:horizontal {
    background: #3498DB;
    border: 2px solid #2980B9;
    width: 18px;
    margin: -6px 0;
    border-radius: 9px;
}

QSlider::handle:horizontal:hover {
    background: #5DADE2;
}

QSlider::sub-page:horizontal {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #3498DB, stop:1 #2ECC71);
    border-radius: 4px;
}
"""

class RosNode(QThread):
    data_arrive_signal = pyqtSignal()
    image_arrive_signal = pyqtSignal()
    connection_status_signal = pyqtSignal(bool, str)
    log_arrive_signal = pyqtSignal(str)
    imu1_arrive_signal = pyqtSignal()
    imu2_arrive_signal = pyqtSignal()
    thruster_arrive_signal = pyqtSignal()
    env_param_arrive_signal = pyqtSignal()
    depth_arrive_signal = pyqtSignal(float)
    force_toque_arrive_signal = pyqtSignal()
    battery_state_arrive_signal = pyqtSignal()
    connection_time_signal = pyqtSignal(str)

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
        self.imu1_subscriber = None
        self.imu2_subscriber = None
        self.thruster_subscriber = None
        self.env_state_subscriber = None
        self.depth_subscriber = None
        self.force_toque_subscriber = None
        self.battery_state_subscriber = None
        
        self.msg = None
        self.image = None
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.is_connected = False # 连接状态标志

        self.connection_start_time = None
        self.connection_timer = None

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
            #取消订阅
            self.cancel_topics_subscriber()

            # 将下述日志同步发布到ROS日志系统中
            rospy.logerr(f"连接失败：{robot_namespace}")
            logger.error(f"连接失败：{robot_namespace}")
            self.connection_status_signal.emit(False, f"连接失败：{robot_namespace}")
            self.is_connected = False
            self.stop_connection_timer()
            return
        
        self.robot_namespace = robot_namespace
        
        # 取消之前的订阅
        self.cancel_topics_subscriber()
        
        # 创建新的发布者和订阅者
        self.simulation_publisher = rospy.Publisher(f'/{robot_namespace}/cmd_vel', Twist, queue_size=1)
        self.simulation_subscriber = rospy.Subscriber(f'/{robot_namespace}/cmd_vel', Twist, callback=self.callback, queue_size=1)
        self.image_subscriber = rospy.Subscriber(f'/{robot_namespace}/camera/image_color', Image_msg, callback=self.image_callback, queue_size=1)
        self.imu1_subscriber = rospy.Subscriber(f'/{robot_namespace}/imu1', Imu, callback=self.imu1_callback, queue_size=10)
        self.imu2_subscriber = rospy.Subscriber(f'/{robot_namespace}/imu2', Imu, callback=self.imu2_callback, queue_size=10)
        self.thruster_subscriber = rospy.Subscriber(f'/{robot_namespace}/thruster_states', JointState, callback=self.thruster_callback, queue_size=1)
        self.env_state_subscriber = rospy.Subscriber(f'/{robot_namespace}/environment_state', Float32MultiArray, callback=self.env_callback, queue_size=1)
        self.depth_subscriber = rospy.Subscriber(f'/{robot_namespace}/pressure', press, callback=self.depth_callback, queue_size=1)
        self.force_toque_subscriber = rospy.Subscriber(f'/{robot_namespace}/force_torque', Float32MultiArray, callback=self.force_toque_callback, queue_size=1)
        self.battery_state_subscriber = rospy.Subscriber(f'/{robot_namespace}/battery_state', Float32MultiArray, callback=self.battery_state_callback, queue_size=1)

        # 将下述日志同步发布到ROS日志系统中
        logger.info(f"连接成功：{robot_namespace}")
        self.connection_status_signal.emit(True, f"连接成功：{robot_namespace}")
        
        self.is_connected = True
        self.start_connection_timer()

    def cancel_topics_subscriber(self):
        if self.simulation_subscriber:
            self.simulation_subscriber.unregister()
        if self.image_subscriber:
            self.image_subscriber.unregister()
        if self.imu1_subscriber:
            self.imu1_subscriber.unregister()
        if self.imu2_subscriber:
            self.imu2_subscriber.unregister()
        if self.thruster_subscriber:
            self.thruster_subscriber.unregister()
        if self.env_state_subscriber:
            self.env_state_subscriber.unregister()
        if self.depth_subscriber:
            self.depth_subscriber.unregister()
        if self.force_toque_subscriber:
            self.force_toque_subscriber.unregister()
        if self.battery_state_subscriber:
            self.battery_state_subscriber.unregister()

    def start_connection_timer(self):
        self.connection_start_time = time.time()
        if self.connection_timer is None:
            self.connection_timer = QTimer()
            self.connection_timer.timeout.connect(self.update_connection_time)
            self.connection_timer.start(1000)  # 每秒更新一次
            rospy.loginfo("连接计时已开始")

    def stop_connection_timer(self):
        if self.connection_timer:
            self.connection_timer.stop()
            self.connection_timer = None
            rospy.loginfo("连接计时已结束")

    def update_connection_time(self):
        if self.connection_start_time is None:
            return

        elapsed_time = int(time.time() - self.connection_start_time)
        hours, remainder = divmod(elapsed_time, 3600)
        minutes, seconds = divmod(remainder, 60)
        time_str = f"{hours:02} : {minutes:02} : {seconds:02}"
        self.connection_time_signal.emit(time_str)
        

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

    def battery_state_callback(self, msg):
        """处理电池状态数据"""
        # 这里假设msg是Float32MultiArray类型
        if len(msg.data) >= 2:
            voltage = msg.data[0]  # 电压
            current = msg.data[1]  # 电流
            self.battery_state_arrive_signal.emit()
            rospy.loginfo(f"电池状态更新: Voltage={voltage:.2f} V, Current={current:.2f} A")
        else:
            rospy.logwarn("接收到的电池状态数据格式不正确")

    def force_toque_callback(self, msg):
        """处理力/力矩数据"""
        # 这里假设msg是Float32类型
        self.force_torque_value = msg.data
        self.force_toque_arrive_signal.emit()
        
    def depth_callback(self, msg):
        current_pressure = msg.fluid_pressure
        #rospy.loginfo(f"深度数据更新: {self.depth_value:.2f} m")
        self.depth_arrive_signal.emit(current_pressure)

    def env_callback(self, msg):
        self.env_state = msg.data
        self.env_param_arrive_signal.emit()
    
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
    
    def imu1_callback(self, msg):
        # 提取欧拉角（弧度）
        orientation_q = msg.orientation
        r = SciRotation.from_quat([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.roll, self.pitch, self.yaw = r.as_euler('xyz', degrees=True)
        # 根据线性加速度计算加速度大小：平方和开根号
        self.acceleration = (msg.linear_acceleration.x**2 + msg.linear_acceleration.y**2 + msg.linear_acceleration.z**2)**0.5
        # 根据角速度计算角速度大小：平方和开根号
        self.angular_velocity = (msg.angular_velocity.x**2 + msg.angular_velocity.y**2 + msg.angular_velocity.z**2)**0.5
        #rospy.loginfo(f"IMU数据到达: Roll={self.roll:.2f}, Pitch={self.pitch:.2f}, Yaw={self.yaw:.2f}")
        self.imu1_arrive_signal.emit()

    def imu2_callback(self, msg):
        # 提取欧拉角（弧度）
        orientation_q = msg.orientation
        r = SciRotation.from_quat([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.roll2, self.pitch2, self.yaw2 = r.as_euler('xyz', degrees=True)
        #rospy.loginfo(f"IMU2数据到达: Roll={self.roll2:.2f}, Pitch={self.pitch2:.2f}, Yaw={self.yaw2:.2f}")
        self.imu2_arrive_signal.emit()

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
        self.setMinimumSize(1600, 900)

        self.robot_info_map = {}
        self.load_robot_config()
        
        self.init_ui()
        self.setWindowTitle("BRICS 水下机器人地面站 v2.0")
        self.setStyleSheet(LIGHT_STYLESHEET)\

        # 设置日志处理器
        #self.setup_logging()
        #sys.stdout = StreamToQt(self.log_signal)
        #sys.stderr = StreamToQt(self.log_signal)

        self.ros_node = RosNode()
        self.ros_node.start()

        self.is_sim = False

        # 设置仿真环境参数
        self.water_density_sim = 1031.0  # kg/m^3
        self.atmospheric_pressure_sim = 0.0  # Pa (stonefish使用的不是标准大气压是表显大气压)
        self.water_density_real = 1025  # kg/m^3 (海水密度)
        self.atmospheric_pressure_real = 101325  # Pa
        self.gravity = 9.81       # m/s^2   
        

        self.setup_connections()

    def load_robot_config(self):
        """从yaml文件加载机器人配置"""
        config_path = os.path.join(os.path.dirname(__file__), 'ip_table.yaml')
        
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                # 加载yaml数据，如果文件为空则返回空字典
                data = yaml.safe_load(f)
                if data:
                    self.robot_info_map = data
                    self.robot_names = list(data.keys())
                    logger.info(f"成功加载配置文件: {config_path}")
                else:
                    self.robot_info_map = {"Default": "127.0.0.1"}
                    logger.warning("配置文件为空，使用默认设置")
                    
        except FileNotFoundError:
            logger.error(f"未找到配置文件: {config_path}")
            self.robot_info_map = {"Error": "No Config File"}
        except Exception as e:
            logger.error(f"读取配置文件出错: {e}")
            self.robot_info_map = {"Error": "Config Error"}

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
    # 顶部状态栏：选择机器人+连接状态+运动模式选择+固件更新
    # =======================================================
    def create_top_bar(self):
        
        top_bar = QFrame()
        top_bar.setFrameShape(QFrame.StyledPanel)
        top_bar.setStyleSheet("QFrame {background-color: white;border-radius: 8px;}")
        top_layout = QHBoxLayout()
        top_bar.setLayout(top_layout)

        ############################ 添加组件到顶部栏 ################################

        select_robot_box = QGroupBox()
        select_robot_layout = QGridLayout()
        select_robot_box.setLayout(select_robot_layout)
        self.select_robot = QLabel('选择机器人:')
        self.select_robot.setStyleSheet("font-weight: bold; font-size: 12px;")
        select_robot_layout.addWidget(self.select_robot,0, 0)

        # 添加机器人选择下拉框
        self.robot_combo = QComboBox()
        robot_names = list(self.robot_info_map.keys())
        self.robot_combo.addItems(robot_names)
        #self.robot_combo.addItems(['robot1', 'robot2', 'robot3'])
        self.robot_combo.setMaximumHeight(30)
        select_robot_layout.addWidget(self.robot_combo, 0, 1)

        # 添加连接按钮
        self.connect_button = QPushButton('连接机器人')
        self.connect_button.setMinimumHeight(30)
        self.connect_button.setStyleSheet("background-color: #28A745; border-color: #1E7E34; color: white; font-weight: bold;")
        select_robot_layout.addWidget(self.connect_button, 0, 2)

        # 第二行：状态
        self.status_label = QLabel("⏳ 等待连接...")
        self.status_label.setProperty("class", "status-label status-warning")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setMaximumHeight(30)
        self.status_label.setStyleSheet("""
            background-color: #FCF3CF;
            color: #9A7D0A;
            border: 2px solid #F4D03F;
            border-radius: 8px;
            padding: 5px;
            font-weight: bold;
        """)
        select_robot_layout.addWidget(self.status_label, 1, 0, 1, 3)

        # 添加显示机器人ip和连接延迟的组件 上层是ip_label 下层是latency_label
        self.ip_label = QLabel('IP地址:')
        self.ip_label.setAlignment(Qt.AlignCenter)

        self.ip_label.setMinimumHeight(10)
        select_robot_layout.addWidget(self.ip_label, 0, 3)
        self.ip_value = QLabel('192.168.x.x')
        self.ip_value.setAlignment(Qt.AlignCenter)
        
        self.ip_value.setMinimumHeight(10)
        select_robot_layout.addWidget(self.ip_value, 0, 4)
        self.latency_label = QLabel('通信延迟:')
        self.latency_label.setAlignment(Qt.AlignCenter)
    
        self.latency_label.setMinimumHeight(10)
        select_robot_layout.addWidget(self.latency_label, 1, 3)
        self.latency_value = QLabel('ping unknown')
        self.latency_value.setAlignment(Qt.AlignCenter)
        
        self.latency_value.setMinimumHeight(10)
        select_robot_layout.addWidget(self.latency_value, 1, 4)
        self.connection_time_label = QLabel('连接时间:')
        self.connection_time_label.setAlignment(Qt.AlignCenter) 
        
        select_robot_layout.addWidget(self.connection_time_label, 0, 5)
        self.connection_time_value = QLabel('00 : 00 : 00')
        self.connection_time_value.setAlignment(Qt.AlignCenter)
        
        self.connection_time_value.setMinimumHeight(10)
        select_robot_layout.addWidget(self.connection_time_value, 0, 6)
        self.stm32_label = QLabel('当前模式:')
        self.stm32_label.setAlignment(Qt.AlignCenter)
        
        self.stm32_label.setMinimumHeight(10)
        select_robot_layout.addWidget(self.stm32_label, 1, 5)
        self.stm32_value = QLabel('F407 Unknown')
        self.stm32_value.setAlignment(Qt.AlignCenter)
        
        self.stm32_value.setMinimumHeight(10)
        select_robot_layout.addWidget(self.stm32_value, 1, 6)

        top_layout.addWidget(select_robot_box)

        # 添加运动模式选择组件
        select_mode_box = QGroupBox()
        select_mode_box.setMaximumHeight(120)
        select_mode_box.setMaximumWidth(300)
        select_mode_layout = QGridLayout()
        select_mode_box.setLayout(select_mode_layout)

        self.mode_label = QLabel('运动模式:')
        self.mode_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        select_mode_layout.addWidget(self.mode_label, 0, 0)
        
        # 添加运动模式选择下拉框
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(['手动模式', '自动定深'])
        self.mode_combo.setMaximumHeight(30)
        select_mode_layout.addWidget(self.mode_combo, 0, 1)

        # 添加确认选择按钮
        self.mode_button = QPushButton('确认模式')
        self.mode_button.setMinimumHeight(30)
        # 背景浅绿 轮廓深绿色
        self.mode_button.setStyleSheet("background-color: #28A745; border-color: #1E7E34; color: white; font-weight: bold;")
        select_mode_layout.addWidget(self.mode_button, 0, 2)
        # 添加选择状态标签
        self.mode_status_label = QLabel('🎮 未激活...')
        self.mode_status_label.setAlignment(Qt.AlignCenter)
        self.mode_status_label.setMaximumHeight(30)
        self.mode_status_label.setProperty("class", "status-label status-error")
        self.mode_status_label.setStyleSheet("""
            background-color: #FADBD8;
            color: #943126;
            border: 2px solid #EC7063;
            border-radius: 6px;
            padding: 5px;
            font-weight: bold;
        """)
        select_mode_layout.addWidget(self.mode_status_label, 1, 0, 1, 3)

        top_layout.addWidget(select_mode_box)

        # 添加固件更新选择组件
        OTA_box = QGroupBox()
        OTA_box.setMaximumHeight(120)
        OTA_box.setMaximumWidth(300)
        OTA_layout = QGridLayout()
        OTA_box.setLayout(OTA_layout)
        self.OTA_label = QLabel('选择固件文件:')
        self.OTA_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        OTA_layout.addWidget(self.OTA_label, 0, 0)
        
        # 添加更新固件选择下拉框
        self.ota_combo = QComboBox()
        self.ota_combo.addItems(['📁 未选择'])
        self.ota_combo.setMaximumHeight(30)
        OTA_layout.addWidget(self.ota_combo, 0, 1)

        # 添加确认选择按钮
        self.ota_button = QPushButton('开始更新')
        self.ota_button.setMinimumHeight(30)
        # 背景浅橙色 边缘深橙色
        self.ota_button.setStyleSheet("background-color: #FFA500; border-color: #FF8C00; color: white; font-weight: bold;")
        OTA_layout.addWidget(self.ota_button, 0, 2)
        # 添加选择状态标签 
        self.ota_status_label = QLabel(' 💤 等待更新指令...')
        self.ota_status_label.setAlignment(Qt.AlignCenter)
        self.ota_status_label.setProperty("class", "status-label status-warning")
        self.ota_status_label.setMaximumHeight(30)
        self.ota_status_label.setStyleSheet("""
            background-color: #D6EAF8;
            color: #1F618D;
            border: 2px solid #5DADE2;
            border-radius: 6px;
            padding: 5px;
            font-weight: bold;
        """)
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
    
    # =================================================================
    # 中央状态栏左侧部分：机器人仪表盘 + IMU数据 + 运动状态 + 环境参数
    # =================================================================
    def create_left_panel(self):
        left_panel = QFrame()
        left_panel.setFrameShape(QFrame.StyledPanel)
        left_panel.setStyleSheet("QFrame {background-color: white;border-radius: 8px;}")
        left_layout = QVBoxLayout()
        left_panel.setLayout(left_layout)

        #############################  仪表盘区域  #############################
        dashboard_layout = QHBoxLayout()
        self.compass_widget = CompassWidget()
        self.horizon_widget = ArtificialHorizonWidget()
        left_dashboard_layout = QVBoxLayout()
        self.compass_label = QLabel('电子罗盘')
        self.compass_label.setStyleSheet("font-weight: bold; font-size: 16px; color: #2980B9;")
        left_dashboard_layout.addWidget(self.compass_label)
        left_dashboard_layout.addWidget(self.compass_widget)
        right_dashboard_layout = QVBoxLayout()
        self.attitude_label = QLabel('姿态仪表')
        self.attitude_label.setStyleSheet("font-weight: bold; font-size: 16px; color: #2980B9;")
        right_dashboard_layout.addWidget(self.attitude_label)
        right_dashboard_layout.addWidget(self.horizon_widget)
        dashboard_layout.addLayout(left_dashboard_layout)
        dashboard_layout.addLayout(right_dashboard_layout)
        left_layout.addLayout(dashboard_layout)

        ############################  双IMU数值显示区域 ############################
        gb_imu = QGroupBox("IMU阵列")
        imu_layout = QGridLayout()
        imu_layout.addWidget(QLabel(""), 0, 0)
        imu_layout.addWidget(QLabel("IMU1"), 0, 1, Qt.AlignCenter)
        imu_layout.addWidget(QLabel("IMU2"), 0, 2, Qt.AlignCenter)
        imu_layout.addWidget(QLabel("误差"), 0, 3, Qt.AlignCenter)

        self.lbl_imu1_r = QLabel("0.0°")
        self.lbl_imu1_p = QLabel("0.0°")
        self.lbl_imu1_y = QLabel("0.0°")
        self.lbl_imu2_r = QLabel("0.0°")
        self.lbl_imu2_p = QLabel("0.0°")
        self.lbl_imu2_y = QLabel("0.0°")
        self.lbl_imu_error_r = QLabel("0.0°")
        self.lbl_imu_error_p = QLabel("0.0°")
        self.lbl_imu_error_y = QLabel("0.0°")

        for lbl in [self.lbl_imu1_r, self.lbl_imu1_p, self.lbl_imu1_y,
                    self.lbl_imu2_r, self.lbl_imu2_p, self.lbl_imu2_y,
                    self.lbl_imu_error_r, self.lbl_imu_error_p, self.lbl_imu_error_y]:
            lbl.setStyleSheet("""
                font-size: 16px;
                font-weight: bold;
                color: #000000;
                font-family: 'Consolas';
                padding: 1px;
            """)

        imu_layout.addWidget(QLabel("Roll/横滚:"), 1, 0)
        imu_layout.addWidget(self.lbl_imu1_r, 1, 1, Qt.AlignCenter)
        imu_layout.addWidget(self.lbl_imu2_r, 1, 2, Qt.AlignCenter)
        imu_layout.addWidget(self.lbl_imu_error_r, 1, 3, Qt.AlignCenter)
        imu_layout.addWidget(QLabel("Pitch/俯仰:"), 2, 0)
        imu_layout.addWidget(self.lbl_imu1_p, 2, 1, Qt.AlignCenter)
        imu_layout.addWidget(self.lbl_imu2_p, 2, 2, Qt.AlignCenter)
        imu_layout.addWidget(self.lbl_imu_error_p, 2, 3, Qt.AlignCenter)
        imu_layout.addWidget(QLabel("Yaw/航向:"), 3, 0)
        imu_layout.addWidget(self.lbl_imu1_y, 3, 1, Qt.AlignCenter)
        imu_layout.addWidget(self.lbl_imu2_y, 3, 2, Qt.AlignCenter)
        imu_layout.addWidget(self.lbl_imu_error_y, 3, 3, Qt.AlignCenter)
        gb_imu.setLayout(imu_layout)
        left_layout.addWidget(gb_imu)

        #################  机器人运动状态：速度 加速度 角速度 角加速度 ########################
        self.lbl_velocity = QLabel("0.00")
        self.lbl_acceleration = QLabel("0.00")
        self.lbl_angular_velocity = QLabel("0.00")
        self.lbl_angular_acceleration = QLabel("0.00")
        "#F89054"
        value_style = """
            font-size: 18px;
            font-weight: bold;
            color: #000000;
            font-family: 'Consolas';
            padding: 1px;
        """
        for lbl in [self.lbl_velocity, self.lbl_acceleration,
                    self.lbl_angular_velocity, self.lbl_angular_acceleration]:
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet(value_style)

        gb_motion = QGroupBox("运动状态")
        gb_motion.setStyleSheet("QGroupBox {margin-top: 10px;}")
        motion_layout = QGridLayout()
        motion_layout.addWidget(QLabel("线速度(m/s)"), 0, 0, Qt.AlignCenter)
        motion_layout.addWidget(QLabel("加速度(m/s²)"), 0, 1, Qt.AlignCenter)
        motion_layout.addWidget(QLabel("角速度(°/s)"), 0, 2, Qt.AlignCenter)
        motion_layout.addWidget(QLabel("角加速度(°/s²)"), 0, 3, Qt.AlignCenter)

        motion_layout.addWidget(self.lbl_velocity, 1, 0, Qt.AlignCenter)
        motion_layout.addWidget(self.lbl_acceleration, 1, 1, Qt.AlignCenter)
        motion_layout.addWidget(self.lbl_angular_velocity, 1, 2, Qt.AlignCenter)
        motion_layout.addWidget(self.lbl_angular_acceleration, 1, 3, Qt.AlignCenter)

        gb_motion.setLayout(motion_layout)
        left_layout.addWidget(gb_motion)

        #####################  环境检测：外部水温 内部水温  ############################
        gb_env = QGroupBox("环境参数")
        env_layout = QGridLayout()
        env_layout.setVerticalSpacing(0)

        self.lbl_temp_water_pressure = QLabel("0.0")
        self.lbl_temp_water = QLabel("0.0")
        self.lbl_temp_internal = QLabel("0.0")
        self.lbl_humi_internal = QLabel("0.0")

        value_style = """
            font-size: 18px;
            font-weight: bold;
            color: #000000;
            font-family: 'Consolas';
            padding: 1px;
        """
        for lbl in [self.lbl_temp_water_pressure, self.lbl_temp_water,
                    self.lbl_temp_internal, self.lbl_humi_internal]:
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet(value_style)
        
        env_layout.addWidget(QLabel("外部水压(kPa)"), 0, 0, Qt.AlignCenter)
        env_layout.addWidget(self.lbl_temp_water_pressure, 1, 0, Qt.AlignCenter)
        env_layout.addWidget(QLabel("外部水温(°C)"), 0, 1, Qt.AlignCenter)
        env_layout.addWidget(self.lbl_temp_water, 1, 1, Qt.AlignCenter)
        env_layout.addWidget(QLabel("舱内温度(°C)"), 0, 2, Qt.AlignCenter)
        env_layout.addWidget(self.lbl_temp_internal, 1, 2, Qt.AlignCenter)
        env_layout.addWidget(QLabel("舱内湿度(%)"), 0, 3, Qt.AlignCenter)
        env_layout.addWidget(self.lbl_humi_internal, 1, 3, Qt.AlignCenter)
        gb_env.setLayout(env_layout)
        left_layout.addWidget(gb_env)

        #####################  深度计： 实际深度 设定深度  ############################

        # 深度计:显示形式是 xx | xx 分别表示实际值和设定值 
        gb_depth = QGroupBox("深度信息")
        depth_layout = QGridLayout()

        # 实际深度标签
        self.lbl_depth_actual = QLabel("0.000")
        self.lbl_depth_actual.setStyleSheet("font-size: 32px; color: #3764F5; font-weight: bold;")
        self.lbl_depth_actual.setAlignment(Qt.AlignCenter)

        # 分隔符 改为黑色粗体竖线
        separator_label = QLabel("|")
        separator_label.setStyleSheet("font-size: 28px; color: #000000; font-weight: bold;")
        separator_label.setAlignment(Qt.AlignCenter)

        # 设定深度标签
        self.lbl_depth_setpoint = QLabel("0.000")
        self.lbl_depth_setpoint.setStyleSheet("font-size: 32px; color: #F1A8CA; font-weight: bold;")
        self.lbl_depth_setpoint.setAlignment(Qt.AlignCenter)

        # 添加到水平布局
        depth_layout.addWidget(self.lbl_depth_actual, 0, 0)
        depth_layout.addWidget(separator_label, 0, 1)
        depth_layout.addWidget(self.lbl_depth_setpoint, 0, 2)

        # 组装深度计布局
        gb_depth.setLayout(depth_layout)
        left_layout.addWidget(gb_depth)

        return left_panel
    
    #========================================================
    # 中央状态栏中间部分：摄像头显示区域
    # =======================================================    
    def create_camera_panel(self):
        camera_panel = QFrame()
        camera_panel.setFrameShape(QFrame.StyledPanel)
        camera_panel.setStyleSheet("QFrame {background-color: white;border-radius: 8px;}")
        camera_layout = QVBoxLayout()
        camera_panel.setLayout(camera_layout)

        # 图片标签
        self.image_label = QLabel('🎥 等待视频流...')
        self.image_label.setMinimumSize(600, 400)
        self.image_label.setStyleSheet("background-color: #2C3E50; border: 2px solid #3498DB; color: white; font-size: 24px;")
        self.image_label.setAlignment(Qt.AlignCenter)
        camera_layout.addWidget(self.image_label)

        return camera_panel
    
    #=======================================================
    # 中央状态栏右边部分：推进器显示+云台角度+推力力矩显示+电池状态
    # =======================================================    
    def create_right_panel(self):
        right_panel = QFrame()
        right_panel.setFrameShape(QFrame.StyledPanel)
        right_panel.setStyleSheet("QFrame {background-color: white;border-radius: 8px;}")
        right_layout = QVBoxLayout()
        right_panel.setLayout(right_layout)

        ########################### 添加推进器显示组件 ###########################
        gb_thruster = QGroupBox("推进器状态")
        self.thruster_widget = RobotHUDWidget()
        gb_thruster.setLayout(QVBoxLayout())
        gb_thruster.layout().addWidget(self.thruster_widget)
        right_layout.addWidget(gb_thruster)

        ########################### 显示二维云台舵机角度：条形式显示 ###########################
        gb_gimbal = QGroupBox("云台角度")
        gimbal_layout = QGridLayout()
        self.angle_1_slider = QSlider(Qt.Horizontal)
        self.angle_1_slider.setMinimum(-90)
        self.angle_1_slider.setMaximum(90)
        self.angle_1_slider.setValue(0)
        self.angle_1_slider.setTickPosition(QSlider.TicksBelow)
        self.angle_1_slider.setTickInterval(30)

        self.angle_2_slider = QSlider(Qt.Horizontal)
        self.angle_2_slider.setMinimum(0)
        self.angle_2_slider.setMaximum(180)
        self.angle_2_slider.setValue(90)
        self.angle_2_slider.setEnabled(True)
        self.angle_2_slider.setTickPosition(QSlider.TicksBelow)  # 显示刻度
        self.angle_2_slider.setTickInterval(10)  #刻度间隔
        gimbal_layout.addWidget(QLabel("旋转角度:"), 0, 0)
        gimbal_layout.addWidget(self.angle_1_slider, 0, 1)
        self.angle_1_value = QLabel("0°")
        self.angle_1_value.setStyleSheet("font-weight: bold; min-width: 40px;")
        gimbal_layout.addWidget(self.angle_1_value, 0, 2)
        gimbal_layout.addWidget(QLabel("俯仰角度:"), 1, 0)
        gimbal_layout.addWidget(self.angle_2_slider, 1, 1)
        self.angle_2_value = QLabel("90°")
        self.angle_2_value.setStyleSheet("font-weight: bold; min-width: 40px;")
        gimbal_layout.addWidget(self.angle_2_value, 1, 2)
        gb_gimbal.setLayout(gimbal_layout)
        right_layout.addWidget(gb_gimbal)   


        ########################### 显示XYZ三个方向上的推力及回转力矩的设定值 ###########################
        gb_thrust = QGroupBox("推力&力矩")
        thrust_layout = QGridLayout()
        self.lbl_thrust_x = QLabel("0.0")
        self.lbl_thrust_y = QLabel("0.0")
        self.lbl_thrust_z = QLabel("0.0")
        self.lbl_torque_x = QLabel("0.0")
        self.lbl_torque_y = QLabel("0.0")
        self.lbl_torque_z = QLabel("0.0")

        value_style = """
            font-size: 20px;
            font-weight: bold;
            color: #000000;
            font-family: 'Consolas';
            padding: 3px;
        """
        for lbl in [self.lbl_thrust_x, self.lbl_thrust_y, self.lbl_thrust_z,
                    self.lbl_torque_x, self.lbl_torque_y, self.lbl_torque_z]:
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet(value_style)

        thrust_layout.addWidget(QLabel(""), 0, 0)
        # 加粗
        thrust_layout.addWidget(QLabel("X方向"), 0, 1, Qt.AlignCenter)
        thrust_layout.addWidget(QLabel("Y方向"), 0, 2, Qt.AlignCenter)
        thrust_layout.addWidget(QLabel("Z方向"), 0, 3, Qt.AlignCenter)
        thrust_layout.addWidget(QLabel("推力 (N):"), 1, 0, Qt.AlignRight)
        thrust_layout.addWidget(self.lbl_thrust_x, 1, 1, Qt.AlignCenter)
        thrust_layout.addWidget(self.lbl_thrust_y, 1, 2, Qt.AlignCenter)
        thrust_layout.addWidget(self.lbl_thrust_z, 1, 3, Qt.AlignCenter)
        thrust_layout.addWidget(QLabel("力矩 (Nm):"), 2, 0, Qt.AlignRight)
        thrust_layout.addWidget(self.lbl_torque_x, 2, 1, Qt.AlignCenter)
        thrust_layout.addWidget(self.lbl_torque_y, 2, 2, Qt.AlignCenter)
        thrust_layout.addWidget(self.lbl_torque_z, 2, 3, Qt.AlignCenter)
        gb_thrust.setLayout(thrust_layout)
        right_layout.addWidget(gb_thrust)

        ########################### 电池状态：电压 电流 剩余容量  ###########################
        gb_battery = QGroupBox("电池状态")
        battery_layout = QGridLayout()
        self.lbl_voltage = QLabel("0.0 V")
        "#3C8AFF"
        self.lbl_voltage.setStyleSheet("font-weight: bold; color: #27AE60;font-size: 20px;")
        self.lbl_current = QLabel("0.0 A")
        self.lbl_current.setStyleSheet("font-weight: bold; color: #FF9743;font-size: 20px;")

        self.battery_progress = QProgressBar()
        self.battery_progress.setValue(85)
        self.battery_progress.setMinimumHeight(25)
        self.battery_progress.setFormat("%p%")
        self.battery_progress.setStyleSheet("""
            QProgressBar {
                border: 2px solid #BDC3C7;
                border-radius: 5px;
                text-align: center;
                color: white;
                background-color: #ECF0F1;
                font-weight: bold;
            }
            QProgressBar::chunk {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                            stop:1 #27AE60, stop:0.5 #F39C12, stop:0 #E74C3C);
                border-radius: 3px;
            }
        """)

        battery_layout.addWidget(QLabel("电池电压:"), 0, 0, Qt.AlignCenter)
        battery_layout.addWidget(self.lbl_voltage, 0, 1)
        battery_layout.addWidget(QLabel("电池电流:"), 0, 2, Qt.AlignCenter)
        battery_layout.addWidget(self.lbl_current, 0, 3)
        battery_layout.addWidget(QLabel("剩余容量:"), 1, 0, Qt.AlignCenter)
        battery_layout.addWidget(self.battery_progress, 1, 1, 1, 3)
        gb_battery.setLayout(battery_layout)
        right_layout.addWidget(gb_battery)

        return right_panel
    
    # =======================================================
    # 底部状态栏：拍照录像按钮 + 日志显示区域
    # =======================================================
    def create_bottom_bar(self):
        bottom_bar = QFrame()
        bottom_bar.setFrameShape(QFrame.StyledPanel)
        bottom_bar.setStyleSheet("QFrame {background-color: white;border-radius: 8px;}")
        bottom_layout = QHBoxLayout()
        bottom_bar.setLayout(bottom_layout)

        bottom_button_layout = QVBoxLayout()
        bottom_layout.addLayout(bottom_button_layout)

        # 开始拍照按钮 背景深灰色
        self.btn_take_photo = QPushButton("📸即刻拍照")
        self.btn_take_photo.setStyleSheet("background-color: #343a40; color: white; font-weight: bold; font-size: 16px; Width: 100px; Height: 32px;")
        bottom_button_layout.addWidget(self.btn_take_photo)

        # 开始录像按钮
        self.btn_start_recording = QPushButton("🎬 开始录像")
        self.btn_start_recording.setStyleSheet("background-color: #343a40; color: white; font-weight: bold; font-size: 16px; Width: 100px; Height: 32px;")
        bottom_button_layout.addWidget(self.btn_start_recording)
        
        # 紧急停止按钮 
        self.btn_estop = QPushButton("🛑 紧急停止")
        self.btn_estop.setStyleSheet("background-color: #d9534f; color: white; font-weight: bold; font-size: 16px; Width: 100px; Height: 32px;")
        bottom_button_layout.addWidget(self.btn_estop)        

        # 日志文本编辑框
        self.log_text_edit = QTextEdit()
        self.log_text_edit.setReadOnly(True)
        self.log_text_edit.setMinimumHeight(150)
        self.log_text_edit.setStyleSheet("border: 3px solid #BDC3C7; font-family: monospace; font-size: 10pt;")
        bottom_layout.addWidget(self.log_text_edit)

        return bottom_bar
    
    # =======================================================
    # 信号与槽函数连接功能实现
    # =======================================================
    def setup_connections(self):
        
        self.connect_button.clicked.connect(self.connect_to_robot)
        self.btn_take_photo.clicked.connect(self.take_photo)
        #self.btn_start_recording.clicked.connect(self.start_recording)
        #self.btn_estop.clicked.connect(self.emergency_stop)

        self.ros_node.image_arrive_signal.connect(self.show_image)
        self.ros_node.connection_status_signal.connect(self.update_connection_status)
        self.ros_node.log_arrive_signal.connect(self.append_log)
        self.ros_node.imu1_arrive_signal.connect(self.show_dashboard)
        self.ros_node.imu2_arrive_signal.connect(self.show_dashboard)
        self.ros_node.thruster_arrive_signal.connect(self.show_thrusters)
        self.ros_node.env_param_arrive_signal.connect(self.update_env_display)
        self.ros_node.depth_arrive_signal.connect(self.update_depth_display)
        self.ros_node.force_toque_arrive_signal.connect(self.update_force_torque_display)
        self.ros_node.connection_time_signal.connect(self.update_connection_time_display)

        # 云台滑块
        self.angle_1_slider.valueChanged.connect(lambda v: self.angle_1_value.setText(f"{v}°"))
        self.angle_2_slider.valueChanged.connect(lambda v: self.angle_2_value.setText(f"{v}°"))

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
            self.status_label.setStyleSheet(""" 
            background-color: #A7F3B9;
            color: #0A8035;
            border: 2px solid #3AA33E;
            border-radius: 8px;
            padding: 5px;
            font-weight: bold;
        """)
            robot_name = self.robot_combo.currentText()
            robot_info = self.robot_info_map.get(robot_name, {})
            ip_address = robot_info.get('ip', '未知IP')
            self.ip_value.setText(ip_address)
            if robot_info.get('pattern', '未知模式') == "sim":
                motion_pattern = "模拟仿真"
                self.is_sim = True
            else:
                motion_pattern = "实机运行"
                self.is_sim = False
            self.stm32_value.setText(motion_pattern)
            rospy.loginfo(f"🤖 已连接至 {robot_name}，IP: {ip_address} 当前模式：{motion_pattern}")
        else:
            self.status_label.setText(message)
            self.status_label.setStyleSheet("""
            background-color: #FADBD8;
            color: #943126;
            border: 2px solid #EC7063;
            border-radius: 8px;
            padding: 5px;
            font-weight: bold;
        """)
            self.is_sim = False
            # 机器人选择区
            self.connection_time_value.setText('00 : 00 : 00')
            self.ip_value.setText('未知IP')
            self.stm32_value.setText('未激活')
            self.show_dashboard_error()
            

    def update_connection_time_display(self, time_str):
        """更新连接时间显示"""
        self.connection_time_value.setText(time_str)
            
    def update_force_torque_display(self, thrusts, torques):
        """更新推力和力矩显示"""
        self.lbl_thrust_x.setText(f"{thrusts[0]:.2f}")
        self.lbl_thrust_y.setText(f"{thrusts[1]:.2f}")
        self.lbl_thrust_z.setText(f"{thrusts[2]:.2f}")
        self.lbl_torque_x.setText(f"{torques[0]:.2f}")
        self.lbl_torque_y.setText(f"{torques[1]:.2f}")
        self.lbl_torque_z.setText(f"{torques[2]:.2f}")
    
    def update_env_display(self, env_data):
        """更新环境参数显示"""
        self.lbl_temp_water_pressure.setText(f"{self.ros_node.current_pressure:.1f}")
        self.lbl_temp_water.setText(f"{env_data['water_temp']:.1f}")
        self.lbl_temp_internal.setText(f"{env_data['internal_temp']:.1f}")
        self.lbl_humi_internal.setText(f"{env_data['internal_humidity']:.1f}")

    def update_depth_display(self, curent_pressure):
        """更新深度计显示"""
        # === 计算公式 ===
        # 深度 = (当前压力 - 大气压) / (密度 * 重力)
        if self.is_sim:
            current_depth = (curent_pressure - self.atmospheric_pressure_sim) / (self.water_density_sim * self.gravity)
        else:
            current_depth = (curent_pressure - self.atmospheric_pressure_real) / (self.water_density_real * self.gravity)
        #rospy.loginfo(f"当前压力: {curent_pressure:.2f} Pa | 当前深度: {current_depth:.4f} m")
        self.lbl_depth_actual.setText(f"{current_depth:.3f}")
        #self.lbl_depth_setpoint.setText(f"{setpoint_depth:.2f}")

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

    def take_photo(self):
        """将当前显示在 QLabel 上的图像保存为文件"""
        # 当前时间戳
        if not self.ros_node.is_connected:
            rospy.logwarn("请先连接到机器人！")
            return
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.image_label.grab().save(f'{SCREENSHOT_PATH}/captured_image_{timestamp}.png', 'PNG')
        rospy.loginfo(f"图片已保存为 captured_image_{timestamp}.png")

    def show_image(self):
        """显示来自ROS图像话题的图像"""
        image = self.ros_node.image
        image = cv2.resize(image, (830, 580))
        show_image = QImage(image, image.shape[1], image.shape[0], image.shape[1]*3, QImage.Format_RGB888)
        self.image_label.setPixmap(QPixmap(show_image))

    def show_dashboard(self):
        """显示机器人仪表数据"""
        self.compass_widget.set_heading(self.ros_node.yaw)
        self.horizon_widget.set_attitude(self.ros_node.roll, self.ros_node.pitch)

        # 更新IMU1数据
        self.lbl_imu1_r.setText(f"{self.ros_node.roll:.1f}°")
        self.lbl_imu1_p.setText(f"{self.ros_node.pitch:.1f}°")
        self.lbl_imu1_y.setText(f"{self.ros_node.yaw:.1f}°")

        # 更新IMU2数据
        self.lbl_imu2_r.setText(f"{self.ros_node.roll2:.1f}°")
        self.lbl_imu2_p.setText(f"{self.ros_node.pitch2:.1f}°")
        self.lbl_imu2_y.setText(f"{self.ros_node.yaw2:.1f}°")

        # 计算并更新误差
        self.lbl_imu_error_r.setText(f"{(self.ros_node.roll - self.ros_node.roll2):.1f}°")
        self.lbl_imu_error_p.setText(f"{(self.ros_node.pitch - self.ros_node.pitch2):.1f}°")
        self.lbl_imu_error_y.setText(f"{(self.ros_node.yaw - self.ros_node.yaw2):.1f}°")

        # 运动状态
        self.lbl_acceleration.setText(f"{self.ros_node.acceleration:.2f}")
        self.lbl_angular_velocity.setText(f"{self.ros_node.angular_velocity:.2f}")

    def show_dashboard_error(self):
        """显示机器人连接失败的GUI数据"""

        # 机器人仪表归零
        self.compass_widget.set_heading(0)
        self.horizon_widget.set_attitude(0, 0)

        # 更新IMU1数据
        self.lbl_imu1_r.setText("0.0°")
        self.lbl_imu1_p.setText("0.0°")
        self.lbl_imu1_y.setText("0.0°")

        # 更新IMU2数据
        self.lbl_imu2_r.setText("0.0°")
        self.lbl_imu2_p.setText("0.0°")
        self.lbl_imu2_y.setText("0.0°")

        # 计算并更新误差
        self.lbl_imu_error_r.setText("0.0°")
        self.lbl_imu_error_p.setText("0.0°")
        self.lbl_imu_error_y.setText("0.0°")

        # 运动状态
        self.lbl_velocity.setText("0.00")
        self.lbl_acceleration.setText("0.00")
        self.lbl_angular_velocity.setText("0.00")
        self.lbl_angular_acceleration.setText("0.00")

        # 环境参数
        self.lbl_temp_water_pressure.setText("0.0")
        self.lbl_temp_water.setText("0.0")
        self.lbl_temp_internal.setText("0.0")  
        self.lbl_humi_internal.setText("0.0")

        # 深度计
        self.lbl_depth_actual.setText("0.000")
        self.lbl_depth_setpoint.setText("0.000")

        # 摄像头显示区
        self.image_label.setText('🎥 等待视频流...')

        # 推进器显示区
        self.thruster_widget.update_thrusts([0]*6)

        # 云台角度
        self.angle_1_slider.setValue(0)
        self.angle_2_slider.setValue(90)

        # 力与力矩显示区
        self.lbl_thrust_x.setText("0.0")
        self.lbl_thrust_y.setText("0.0")
        self.lbl_thrust_z.setText("0.0")
        self.lbl_torque_x.setText("0.0")
        self.lbl_torque_y.setText("0.0")
        self.lbl_torque_z.setText("0.0")

        # 电池状态
        self.lbl_voltage.setText("0.0 V")
        self.lbl_current.setText("0.0 A")
        self.battery_progress.setValue(0)

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
