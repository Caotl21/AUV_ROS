#!/usr/bin/env python3
import rospy
import serial
import struct
from time import sleep
from underwater_msgs.msg import ThrustersPWM, SensorData

class UnderwaterSerialNode:
    def __init__(self):
        # 从参数服务器获取串口配置
        port = rospy.get_param('~port', '/dev/stm32')
        baudrate = rospy.get_param('~baudrate', 9600)
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
        self.START_BYTE_1 = 0xAA
        self.START_BYTE_2 = 0xBB
        self.END_BYTE_1 = 0xCC
        self.END_BYTE_2 = 0xDD
        
        # CRC8查找表
        self.crc8_table = self._generate_crc8_table()
        
        rospy.loginfo("水下机器人串口通信节点已启动")
        
        # 启动接收线程
        self.receive_loop()
    
    def _generate_crc8_table(self):
        """生成CRC8查找表"""
        table = []
        for i in range(256):
            crc = i
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x07
                else:
                    crc <<= 1
                crc &= 0xFF
            table.append(crc)
        return table
    
    def calculate_crc8(self, data):
        """计算CRC8校验值"""
        crc = 0
        for byte in data:
            crc = self.crc8_table[crc ^ byte]
        return crc
    
    def is_connected(self):
        """检查串口是否连接"""
        return self.serial is not None and self.serial.isOpen()
    
    def encode_data(self, data_type, payload):
        """编码数据为帧格式"""
        # 帧结构: [起始字节][数据类型][数据长度][数据负载][CRC8校验][结束字节]
        frame = bytearray()
        frame.append(self.START_BYTE_1)
        frame.append(self.START_BYTE_2)
        
        frame.append(data_type)
        frame.append(len(payload))
        frame.extend(payload)
        
        # 计算CRC8校验
        crc8 = self.calculate_crc8(frame[2:])  # 从数据类型开始计算
        frame.append(crc8)
        frame.append(self.END_BYTE_1)
        frame.append(self.END_BYTE_2)
        
        return frame
    
    '''def decode_data(self, frame):
        """解码接收到的帧数据"""
        if len(frame) < 5:  # 最小帧长度检查
            return None, None
        
        if frame[0] != self.START_BYTE or frame[-1] != self.END_BYTE:
            return None, None
        
        # 验证CRC8
        received_crc = frame[-2]
        calculated_crc = self.calculate_crc8(frame[1:-2])  # 从数据类型到数据负载结束
        
        #if calculated_crc != received_crc:
        #    rospy.logwarn("CRC8校验失败")
        #    return None, None
        
        data_type = frame[1]
        data_length = frame[2]
        payload = frame[3:3+data_length]
        
        return data_type, payload, data_length'''
    def decode_data(self, frame):
        """解码接收到的帧数据"""
        if len(frame) < 5:  # 最小帧长度检查
            rospy.logwarn(f"帧长度不足: {len(frame)}字节")
            return None, None, None
        
        if frame[0] != self.START_BYTE:
            rospy.logwarn(f"起始字节错误: 0x{frame[0]:02X}, 期望: 0x{self.START_BYTE:02X}")
            return None, None, None
            
        if frame[-1] != self.END_BYTE:
            rospy.logwarn(f"结束字节错误: 0x{frame[-1]:02X}, 期望: 0x{self.END_BYTE:02X}")
            return None, None, None
        
        # 验证CRC8 (暂时注释掉)
        # received_crc = frame[-2]
        # calculated_crc = self.calculate_crc8(frame[1:-2])  # 从数据类型到数据负载结束
        # if calculated_crc != received_crc:
        #     rospy.logwarn("CRC8校验失败")
        #     return None, None, None
        
        data_type = frame[1]
        data_length = frame[2]
        
        # 检查数据长度是否匹配
        expected_length = 3 + data_length + 2  # 头3字节 + 数据长度 + CRC和结束字节
        
        if len(frame) != expected_length:
            rospy.logwarn(f"数据长度不匹配: 期望{expected_length}字节, 实际{len(frame)}字节")
            return None, None, None
        
        payload = frame[3:3+data_length]
        rospy.loginfo(f"数据长度匹配: 期望{expected_length}字节, 实际{len(frame)}字节")
        rospy.loginfo(f"成功解码数据包: 类型0x{data_type:02X}, 长度{data_length}字节")
        return data_type, data_length, payload
    
    def send_thruster_data(self, pwm_values):
        """发送推进器PWM数据到STM32"""
        if not self.is_connected():
            rospy.logwarn("串口未连接，无法发送数据")
            return False
        
        try:
            # 将6个int16 PWM值打包为字节数据
            payload = bytearray()
            rospy.loginfo(f"PWM值详细编码:")
            for i, pwm in enumerate(pwm_values):
                # 确保值在int16范围内
                pwm = max(-32768, min(32767, pwm))
                # 大端序打包int16 (改为'>h')
                pwm_bytes = struct.pack('>H', pwm)
                payload.extend(pwm_bytes)
                rospy.loginfo(f"  PWM{i+1}: {pwm} (0x{pwm:04X}) -> {[f'0x{b:02X}' for b in pwm_bytes]}")
            
            # 编码数据帧 (数据类型1表示推进器数据)
            frame = self.encode_data(1, payload)
            rospy.loginfo(f"完整数据包: {[f'0x{b:02X}' for b in frame]}")
            
            # 发送数据
            self.serial.write(frame)
            rospy.loginfo(f"发送推进器数据: {[f'0x{b:02X}' for b in frame]}")
            return True
        except Exception as e:
            rospy.logerr(f"发送数据失败: {e}")
            return False
    
    def process_sensor_data(self, length, payload):
        """处理接收到的传感器数据"""
        try:
            # 传感器数据结构: 6个int16_t类型，每个2字节，共12字节
            # 顺序: [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, depth, temperature
                         
            if len(payload) == length:  # 8个int16_t = 16字节）
                accel_x = struct.unpack('>h', payload[0:2])[0]
                accel_y = struct.unpack('>h', payload[2:4])[0]
                accel_z = struct.unpack('>h', payload[4:6])[0]
                gyro_x = struct.unpack('>h', payload[6:8])[0]
                gyro_y = struct.unpack('>h', payload[8:10])[0]
                gyro_z = struct.unpack('>h', payload[10:12])[0]
                depth = struct.unpack('>h', payload[12:14])[0]
                temperature = struct.unpack('>h', payload[14:16])[0]
                
                sensor_msg = SensorData()
                sensor_msg.accel_x = accel_x
                sensor_msg.accel_y = accel_y
                sensor_msg.accel_z = accel_z
                sensor_msg.gyro_x = gyro_x
                sensor_msg.gyro_y = gyro_y
                sensor_msg.gyro_z = gyro_z
                sensor_msg.depth = depth
                sensor_msg.temperature = temperature
                sensor_msg.header.stamp = rospy.Time.now()
                
                self.sensor_pub.publish(sensor_msg)
                
                rospy.loginfo(f"传感器数据 - 加速度: ({accel_x}, {accel_y}, {accel_z}) "
                            f"陀螺仪: ({gyro_x}, {gyro_y}, {gyro_z}) "
                            f"深度: {depth}, 温度: {temperature}")
                            
            else:
                rospy.logwarn(f"传感器数据长度不足: {len(payload)}字节，期望至少16字节")
            
        except Exception as e:
            rospy.logwarn(f"处理传感器数据失败: {e}")
    
    def thruster_callback(self, msg):
        """处理接收到的推进器PWM指令"""
        # ThrustersPWM消息包含6个int16类型的字段
        pwm_values = [msg.pwm1, msg.pwm2, msg.pwm3, msg.pwm4, msg.pwm5, msg.pwm6]
        # 先打印再发送
        rospy.loginfo(f"接收到的推进器PWM指令: {pwm_values}")
        self.send_thruster_data(pwm_values)
    
    def receive_loop(self):
        """接收数据的循环"""
        buffer = bytearray()
        in_frame = False
        
        while not rospy.is_shutdown() and self.is_connected():
            try:
                # 读取可用数据
                #data = self.serial.read(self.serial.in_waiting or 1)
                #data = self.serial.read(self.serial.in_waiting)
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                else:
                    data = b''  # 空字节
                if(len(data) != 0):
                    rospy.loginfo(f"收到{len(data)}字节数据")
                if not data:
                    continue
                
                for byte in data:
                    if byte == self.START_BYTE:
                        buffer = bytearray()
                        buffer.append(byte)
                        in_frame = True
                    elif byte == self.END_BYTE and in_frame:
                        buffer.append(byte)
                        # 处理完整帧
                        data_type, data_length, payload = self.decode_data(buffer)
                        if data_type is not None:
                            if data_type == 2:  # 数据类型2表示传感器数据
                                self.process_sensor_data(data_length,payload)
                        in_frame = False
                    elif in_frame:
                        buffer.append(byte)

                # 不在帧中时
                #if not in_frame and len(buffer) > 0:
                #    buffer.clear()
                
                # 防止缓冲区过大
                if len(buffer) > 50:
                    rospy.logwarn("接收缓冲区溢出，清空缓冲区")
                    buffer.clear()
                    in_frame = False
                    
            except Exception as e:
                rospy.logerr(f"接收数据时出错: {e}")
                buffer.clear()
                in_frame = False
            
            sleep(0.001)  # 短暂休眠以减少CPU使用
    '''def receive_loop(self):
        """接收数据的循环"""
        buffer = bytearray()
        in_frame = False
        
        rospy.loginfo("开始接收数据循环...")
        
        while not rospy.is_shutdown() and self.is_connected():
            try:
                # 读取可用数据
                data = self.serial.read(self.serial.in_waiting or 1)
                if data:
                    rospy.logdebug(f"收到{len(data)}字节数据")
                    
                for byte in data:
                    if byte == self.START_BYTE:
                        buffer = bytearray()
                        buffer.append(byte)
                        in_frame = True
                        rospy.logdebug("检测到起始字节")
                    elif byte == self.END_BYTE and in_frame:
                        buffer.append(byte)
                        rospy.logdebug(f"检测到结束字节，完整帧长度: {len(buffer)}")
                        
                        # 处理完整帧
                        result = self.decode_data(buffer)
                        if result[0] is not None:
                            data_type, data_length, payload = result
                            if data_type == 2:  # 数据类型2表示传感器数据
                                self.process_sensor_data(data_length, payload)
                            else:
                                rospy.loginfo(f"收到其他类型数据包: 0x{data_type:02X}")
                        else:
                            rospy.logwarn("数据包解码失败")
                            
                        in_frame = False
                    elif in_frame:
                        buffer.append(byte)
                        # 防止缓冲区过大
                        if len(buffer) > 50:
                            rospy.logwarn("缓冲区过大，重置状态")
                            buffer.clear()
                            in_frame = False
                
                # 防止缓冲区过大（不在帧中时）
                if not in_frame and len(buffer) > 0:
                    buffer.clear()
                    
                sleep(0.001)  # 短暂休眠以减少CPU使用
                    
            except Exception as e:
                rospy.logerr(f"接收数据时出错: {e}")
                buffer.clear()
                in_frame = False
                sleep(0.1)'''
    
    def close(self):
        """关闭串口"""
        if self.serial and self.serial.isOpen():
            self.serial.close()
            rospy.loginfo("串口已关闭")

def main():
    rospy.init_node('underwater_serial_node', anonymous=True)
    
    node = UnderwaterSerialNode()
    
    # 设置关闭时的清理操作
    rospy.on_shutdown(node.close)
    
    rospy.loginfo("水下机器人串口通信节点运行中...")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
