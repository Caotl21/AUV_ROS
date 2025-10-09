#!/usr/bin/env python3
import rospy
import serial
from time import sleep
from std_msgs.msg import String

class SerialCommunicationNode:
    def __init__(self):
        # 从参数服务器获取串口配置
        port = rospy.get_param('~port', '/dev/ttyUSB0')
        baudrate = rospy.get_param('~baudrate', 9600)
        mode = rospy.get_param('~mode', 'dec')
        
        self.serial = None
        self.mode = mode
        
        # 初始化串口
        try:
            self.serial = serial.Serial(port, baudrate, timeout=0.5)
            if self.serial.isOpen():
                rospy.loginfo(f"串口打开成功: {port}, 波特率: {baudrate}")
            else:
                rospy.logerr("串口打开失败")
                return
        except serial.SerialException as e:
            rospy.logerr(f"串口打开失败: {e}")
            return
        
        # 创建订阅者
        self.sub = rospy.Subscriber('servo_serial', String, self.data_callback)
        
        # 创建发布者（可选，用于发布接收到的数据）
        self.pub = rospy.Publisher('serial_received', String, queue_size=10)
        
        rospy.loginfo(f"串口通信节点已启动，模式: {'十进制' if mode == 'dec' else '十六进制'}")
    
    def is_connected(self):
        """检查串口是否连接"""
        return self.serial is not None and self.serial.isOpen()
    
    def recv(self):
        """接收数据"""
        if not self.is_connected():
            return None, None
            
        data = self.serial.read_all()
        if data:
            # 返回十六进制格式和原始字节格式
            big_int = int.from_bytes(data, byteorder='big')
            return big_int, data.hex()
        return None, None
    
    def send_decimal(self, decimal_data):
        """发送十进制整数"""
        try:
            num = int(decimal_data)
            # 将整数转换为字节 (使用1字节表示，大端序)
            send_data = num.to_bytes(1, byteorder='big')
            self.serial.write(send_data)
            rospy.loginfo(f"发送成功: 十进制 {num} -> 字节 {send_data}")
            return send_data
        except ValueError:
            rospy.logwarn("输入的不是有效数字")
            return None
    
    def send_hex(self, hex_data):
        """发送十六进制字符串"""
        try:
            send_data = bytes.fromhex(hex_data)
            self.serial.write(send_data)
            rospy.loginfo(f"发送成功: 十六进制 {hex_data} -> 字节 {send_data}")
            return send_data
        except ValueError:
            rospy.logwarn("输入的不是有效十六进制数")
            return None
    
    def data_callback(self, msg):
        """处理接收到的ROS消息"""
        if not self.is_connected():
            rospy.logwarn("串口未连接，无法发送数据")
            return
            
        data = msg.data.strip()
        
        if not data:
            rospy.logwarn("接收到的数据为空")
            return
        
        # 根据模式发送数据
        if self.mode == 'dec':
            sent_data = self.send_decimal(data)
        elif self.mode == 'hex':
            sent_data = self.send_hex(data)
        else:
            rospy.logwarn(f"未知的模式: {self.mode}")
            return
        
        # 等待并接收返回数据
        sleep(0.1)
        dec_data, hex_data = self.recv()
        
        if dec_data is not None:
            # 发布接收到的数据
            if self.mode == 'dec':
                result_msg = String(f"接收到的数据 (十进制): {dec_data}")
            else:
                result_msg = String(f"接收到的数据 (十六进制): {hex_data}")
            
            self.pub.publish(result_msg)
    
    def close(self):
        """关闭串口"""
        if self.serial and self.serial.isOpen():
            self.serial.close()
            rospy.loginfo("串口已关闭")

def main():
    rospy.init_node('serial_communication_node', anonymous=True)
    
    node = SerialCommunicationNode()
    
    # 设置关闭时的清理操作
    rospy.on_shutdown(node.close)
    
    rospy.loginfo("串口通信节点运行中...")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
