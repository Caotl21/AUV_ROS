import rospy
import serial
import struct
from time import sleep
from std_msgs.msg import String
from rov_msgs.msg import ThrustersPWM, SensorData  # 自定义消息类型

class SerialCommunicationNode:
    def __init__(self):
        # 从参数服务器获取串口配置
        port = rospy.get_param('~port', '/dev/ttyUSB0')
        baudrate = rospy.get_param('~baudrate', 115200)  # 提高波特率以适应更多数据
        self.timeout = rospy.get_param('~timeout', 0.5)
        
        self.serial = None
        
        # 初始化串口
        try:
            self.serial = serial.Serial(port, baudrate, timeout=self.timeout)
            if self.serial.isOpen():
                rospy.loginfo(f"串口打开成功: {port}, 波特率: {baudrate}")
            else:
                rospy.logerr("串口打开失败")
                return
        except serial.SerialException as e:
            rospy.logerr(f"串口打开失败: {e}")
            return
        
        # 创建订阅者 - 订阅推进器PWM指令
        self.thruster_sub = rospy.Subscriber('thrusters_pwm', ThrustersPWM, self.thruster_callback)
        
        # 创建发布者 - 发布传感器数据
        self.sensor_pub = rospy.Publisher('sensor_data', SensorData, queue_size=10)
        
        # 定义通信协议
        self.START_BYTE = 0xAA
        self.END_BYTE = 0x55
        self.ESCAPE_BYTE = 0x5C
        
        rospy.loginfo("多字节串口通信节点已启动")
        
        # 启动接收线程
        self.receive_loop()
    
    def is_connected(self):
        """检查串口是否连接"""
        return self.serial is not None and self.serial.isOpen()
    
    def encode_data(self, data_type, payload):
        """编码数据为帧格式"""
        # 帧结构: [起始字节][数据类型][数据长度][数据负载][CRC校验][结束字节]
        frame = bytearray()
        frame.append(self.START_BYTE)
        frame.append(data_type)
        frame.append(len(payload))
        frame.extend(payload)
        
        # 计算CRC (简单异或校验)
        crc = 0
        for byte in frame[1:]:  # 从数据类型开始计算
            crc ^= byte
        frame.append(crc)
        frame.append(self.END_BYTE)
        
        return frame
    
    def decode_data(self, frame):
        """解码接收到的帧数据"""
        if len(frame) < 5:  # 最小帧长度检查
            return None, None
        
        if frame[0] != self.START_BYTE or frame[-1] != self.END_BYTE:
            return None, None
        
        # 验证CRC
        crc = 0
        for byte in frame[1:-2]:  # 从数据类型到数据负载结束
            crc ^= byte
        
        if crc != frame[-2]:
            rospy.logwarn("CRC校验失败")
            return None, None
        
        data_type = frame[1]
        data_length = frame[2]
        payload = frame[3:3+data_length]
        
        return data_type, payload
    
    def send_thruster_data(self, pwm_values):
        """发送推进器PWM数据到STM32"""
        if not self.is_connected():
            rospy.logwarn("串口未连接，无法发送数据")
            return False
        
        try:
            # 将6个PWM值打包为字节数据 (假设每个PWM值是0-255的整数)
            payload = bytearray()
            for pwm in pwm_values:
                payload.append(max(0, min(255, pwm)))  # 确保值在0-255范围内
            
            # 编码数据帧 (数据类型1表示推进器数据)
            frame = self.encode_data(1, payload)
            
            # 发送数据
            self.serial.write(frame)
            rospy.logdebug(f"发送推进器数据: {[f'0x{b:02X}' for b in frame]}")
            return True
        except Exception as e:
            rospy.logerr(f"发送数据失败: {e}")
            return False
    
    def process_sensor_data(self, payload):
        """处理接收到的传感器数据"""
        try:
            # 假设传感器数据包含: 深度(4字节float), 温度(2字节short), 湿度(2字节short)
            # 根据您的实际传感器数据格式进行修改
            if len(payload) >= 8:
                depth = struct.unpack('f', payload[0:4])[0]
                temperature = struct.unpack('h', payload[4:6])[0] / 10.0  # 假设温度以0.1°C为单位
                humidity = struct.unpack('h', payload[6:8])[0] / 10.0     # 假设湿度以0.1%为单位
                
                # 创建并发布传感器消息
                sensor_msg = SensorData()
                sensor_msg.depth = depth
                sensor_msg.temperature = temperature
                sensor_msg.humidity = humidity
                sensor_msg.header.stamp = rospy.Time.now()
                
                self.sensor_pub.publish(sensor_msg)
                rospy.logdebug(f"发布传感器数据: 深度={depth}, 温度={temperature}, 湿度={humidity}")
        except Exception as e:
            rospy.logwarn(f"处理传感器数据失败: {e}")
    
    def thruster_callback(self, msg):
        """处理接收到的推进器PWM指令"""
        # 假设ThrustersPWM消息包含6个int16类型的字段: pwm1到pwm6
        pwm_values = [msg.pwm1, msg.pwm2, msg.pwm3, msg.pwm4, msg.pwm5, msg.pwm6]
        # 现将topic收到的消息打印出来再发送
        rospy.loginfo(f"接收到的推进器PWM指令: {pwm_values}")
        self.send_thruster_data(pwm_values)
    
    def receive_loop(self):
        """接收数据的循环"""
        buffer = bytearray()
        in_frame = False
        escape_next = False
        
        while not rospy.is_shutdown() and self.is_connected():
            try:
                # 读取可用数据
                data = self.serial.read(self.serial.in_waiting or 1)
                if not data:
                    continue
                
                for byte in data:
                    if escape_next:
                        buffer.append(byte)
                        escape_next = False
                        continue
                    
                    if byte == self.START_BYTE:
                        buffer = bytearray()
                        buffer.append(byte)
                        in_frame = True
                    elif byte == self.END_BYTE and in_frame:
                        buffer.append(byte)
                        # 处理完整帧
                        data_type, payload = self.decode_data(buffer)
                        if data_type is not None:
                            if data_type == 2:  # 假设数据类型2表示传感器数据
                                self.process_sensor_data(payload)
                            # 可以添加其他数据类型的处理
                        in_frame = False
                    elif byte == self.ESCAPE_BYTE and in_frame:
                        escape_next = True
                    elif in_frame:
                        buffer.append(byte)
                
                # 防止缓冲区过大
                if len(buffer) > 256:
                    rospy.logwarn("接收缓冲区溢出，清空缓冲区")
                    buffer = bytearray()
                    in_frame = False
                    
            except Exception as e:
                rospy.logerr(f"接收数据时出错: {e}")
                buffer = bytearray()
                in_frame = False
            
            sleep(0.001)  # 短暂休眠以减少CPU使用
    
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
    
    rospy.loginfo("多字节串口通信节点运行中...")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
