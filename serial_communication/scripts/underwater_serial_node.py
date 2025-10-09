#!/usr/bin/env python3
import rospy
import serial
import struct
from time import sleep
from enum import Enum
from underwater_msgs.msg import ThrustersPWM, SensorData, ServoPWM

class ReceiveState(Enum):
    WAIT_START_1 = 0    # 等待第一个起始字节 0xAA
    WAIT_START_2 = 1    # 等待第二个起始字节 0xBB
    WAIT_TYPE = 2       # 等待数据类型字节
    WAIT_LENGTH = 3     # 等待数据长度字节
    WAIT_DATA = 4       # 等待数据负载
    WAIT_CRC = 5        # 等待CRC校验字节
    WAIT_END_1 = 6      # 等待第一个结束字节 0xCC
    WAIT_END_2 = 7      # 等待第二个结束字节 0xDD

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
        self.servo_sub = rospy.Subscriber('servo_pwm', ServoPWM, self.servo_callback)
        
        # 创建发布者 - 发布传感器数据
        self.sensor_pub = rospy.Publisher('sensor_data', SensorData, queue_size=10)
        
        # 定义通信协议
        self.START_BYTE_1 = 0xAA
        self.START_BYTE_2 = 0xBB
        self.END_BYTE_1 = 0xCC
        self.END_BYTE_2 = 0xDD

        # 状态机相关变量
        self.rx_state = ReceiveState.WAIT_START_1
        self.rx_data_type = 0
        self.rx_data_length = 0
        self.rx_payload = bytearray()
        self.rx_crc = 0
        self.rx_data_count = 0
        
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
        
    def send_servo_data(self, pwm_values):
        """发送舵机PWM数据到STM32"""
        if not self.is_connected():
            rospy.logwarn("串口未连接，无法发送数据")
            return False
        
        try:
            # 将2个int16 PWM值打包为字节数据
            payload = bytearray()
            rospy.loginfo(f"PWM值详细编码:")
            for i, pwm in enumerate(pwm_values):
                # 确保值在int16范围内
                pwm = max(-32768, min(32767, pwm))
                # 大端序打包int16 (改为'>h')
                pwm_bytes = struct.pack('>H', pwm)
                payload.extend(pwm_bytes)
                rospy.loginfo(f"  PWM{i+1}: {pwm} (0x{pwm:04X}) -> {[f'0x{b:02X}' for b in pwm_bytes]}")
            
            # 编码数据帧 (数据类型3表示舵机数据)
            frame = self.encode_data(3, payload)
            rospy.loginfo(f"完整数据包: {[f'0x{b:02X}' for b in frame]}")
            
            # 发送数据
            self.serial.write(frame)
            rospy.loginfo(f"发送舵机数据: {[f'0x{b:02X}' for b in frame]}")
            return True
        except Exception as e:
            rospy.logerr(f"发送数据失败: {e}")
            return False
    
    def process_sensor_data(self, length, payload):
        """处理接收到的传感器数据"""
        try:
            # 传感器数据结构: 15个int16_t类型，每个2字节，共30字节
            # 顺序: [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, 
            # angle_x, angle_y, angle_z, depth, temperature, humidity
                         
            if len(payload) == length:  # 8个int16_t = 16字节）
                accel_x = struct.unpack('>h', payload[0:2])[0]
                accel_y = struct.unpack('>h', payload[2:4])[0]
                accel_z = struct.unpack('>h', payload[4:6])[0]
                gyro_x = struct.unpack('>h', payload[6:8])[0]
                gyro_y = struct.unpack('>h', payload[8:10])[0]
                gyro_z = struct.unpack('>h', payload[10:12])[0]
                mag_x = struct.unpack('>h', payload[12:14])[0]
                mag_y = struct.unpack('>h', payload[14:16])[0]
                mag_z = struct.unpack('>h', payload[16:18])[0]
                angle_x = struct.unpack('>h', payload[18:20])[0]
                angle_y = struct.unpack('>h', payload[20:22])[0]
                angle_z = struct.unpack('>h', payload[22:24])[0]
                depth = struct.unpack('>h', payload[24:26])[0]
                temperature = struct.unpack('>h', payload[26:28])[0]
                humidity = struct.unpack('>h', payload[28:30])[0]
                
                sensor_msg = SensorData()
                sensor_msg.accel_x = accel_x
                sensor_msg.accel_y = accel_y
                sensor_msg.accel_z = accel_z
                sensor_msg.gyro_x = gyro_x
                sensor_msg.gyro_y = gyro_y
                sensor_msg.gyro_z = gyro_z
                sensor_msg.mag_x = mag_x
                sensor_msg.mag_y = mag_y
                sensor_msg.mag_z = mag_z
                sensor_msg.angle_x = angle_x
                sensor_msg.angle_y = angle_y
                sensor_msg.angle_z = angle_z
                sensor_msg.depth = depth
                sensor_msg.temperature = temperature
                sensor_msg.humidity = humidity
                sensor_msg.header.stamp = rospy.Time.now()
                
                self.sensor_pub.publish(sensor_msg)
                
                rospy.loginfo(f"传感器数据 - 加速度: ({accel_x}, {accel_y}, {accel_z}) "
                            f"陀螺仪: ({gyro_x}, {gyro_y}, {gyro_z}) "
                            f"磁力计: ({mag_x}, {mag_y}, {mag_z}) "
                            f"角度: ({angle_x}, {angle_y}, {angle_z}) "
                            f"深度: {depth}, 温度: {temperature}, 湿度: {humidity}")
                            
            else:
                rospy.logwarn(f"传感器数据长度不足: {len(payload)}字节，期望至少30字节")
            
        except Exception as e:
            rospy.logwarn(f"处理传感器数据失败: {e}")
    
    def thruster_callback(self, msg):
        """处理接收到的推进器PWM指令"""
        # ThrustersPWM消息包含6个int16类型的字段
        pwm_values = [msg.pwm1, msg.pwm2, msg.pwm3, msg.pwm4, msg.pwm5, msg.pwm6]
        # 先打印再发送
        rospy.loginfo(f"接收到的推进器PWM指令: {pwm_values}")
        self.send_thruster_data(pwm_values)

    def servo_callback(self, msg):
        """处理接收到的舵机PWM指令"""
        # ServoPWM消息包含2个int16类型的字段
        pwm_values = [msg.pwm_yaw, msg.pwm_pitch]
        # 先打印再发送
        rospy.loginfo(f"接收到的舵机PWM指令: {pwm_values}")
        self.send_servo_data(pwm_values)
    
    def reset_state_machine(self):
            """重置状态机"""
            self.rx_state = ReceiveState.WAIT_START_1
            self.rx_data_type = 0
            self.rx_data_length = 0
            self.rx_payload.clear()
            self.rx_crc = 0
            self.rx_data_count = 0

    def handle_complete_packet(self):
        """处理完整接收的数据包"""
        try:
            # 可选：验证CRC
            # calculated_crc = self.calculate_crc8([self.rx_data_type, self.rx_data_length] + list(self.rx_payload))
            # if calculated_crc != self.rx_crc:
            #     rospy.logwarn("CRC校验失败")
            #     return
            
            rospy.loginfo(f"接收到完整数据包: 类型=0x{self.rx_data_type:02X}, 长度={self.rx_data_length}")
            
            if self.rx_data_type == 2:  # 传感器数据
                self.process_sensor_data(self.rx_data_length, self.rx_payload)
            else:
                rospy.loginfo(f"收到其他类型数据包: 0x{self.rx_data_type:02X}")
                
        except Exception as e:
            rospy.logerr(f"处理数据包时出错: {e}")

    def receive_loop(self):
        """使用状态机接收数据的循环"""
        while not rospy.is_shutdown() and self.is_connected():
            try:
                # 逐字节读取
                if self.serial.in_waiting > 0:
                    data = self.serial.read(1)
                    if data:
                        self.process_byte(data[0])
                else:
                    sleep(0.001)  # 短暂休眠避免CPU占用过高
                    
            except Exception as e:
                rospy.logerr(f"接收数据时出错: {e}")
                self.reset_state_machine()
    
    def process_byte(self, byte):
        """状态机处理单个字节"""
        if self.rx_state == ReceiveState.WAIT_START_1:
            if byte == self.START_BYTE_1:
                self.rx_state = ReceiveState.WAIT_START_2
                rospy.logdebug("收到起始字节1")
            # 其他字节直接丢弃
            
        elif self.rx_state == ReceiveState.WAIT_START_2:
            if byte == self.START_BYTE_2:
                self.rx_state = ReceiveState.WAIT_TYPE
                rospy.logdebug("收到起始字节2")
            else:
                # 不是期望的第二个起始字节，重新开始
                if byte == self.START_BYTE_1:
                    self.rx_state = ReceiveState.WAIT_START_2  # 可能是新帧的开始
                else:
                    self.rx_state = ReceiveState.WAIT_START_1
                    
        elif self.rx_state == ReceiveState.WAIT_TYPE:
            self.rx_data_type = byte
            self.rx_state = ReceiveState.WAIT_LENGTH
            rospy.logdebug(f"收到数据类型: 0x{byte:02X}")
            
        elif self.rx_state == ReceiveState.WAIT_LENGTH:
            self.rx_data_length = byte
            self.rx_data_count = 0
            self.rx_payload.clear()
            if self.rx_data_length == 0:
                # 没有数据负载，直接等待CRC
                self.rx_state = ReceiveState.WAIT_CRC
            else:
                self.rx_state = ReceiveState.WAIT_DATA
            rospy.logdebug(f"收到数据长度: {byte}")
            
        elif self.rx_state == ReceiveState.WAIT_DATA:
            self.rx_payload.append(byte)
            self.rx_data_count += 1
            if self.rx_data_count >= self.rx_data_length:
                self.rx_state = ReceiveState.WAIT_CRC
                rospy.logdebug(f"数据负载接收完成: {len(self.rx_payload)}字节")
                
        elif self.rx_state == ReceiveState.WAIT_CRC:
            self.rx_crc = byte
            self.rx_state = ReceiveState.WAIT_END_1
            rospy.logdebug(f"收到CRC: 0x{byte:02X}")
            
        elif self.rx_state == ReceiveState.WAIT_END_1:
            if byte == self.END_BYTE_1:
                self.rx_state = ReceiveState.WAIT_END_2
                rospy.logdebug("收到结束字节1")
            else:
                # 结束字节错误，重新开始
                rospy.logwarn("结束字节1错误，重新开始")
                self.reset_state_machine()
                if byte == self.START_BYTE_1:
                    self.rx_state = ReceiveState.WAIT_START_2
                    
        elif self.rx_state == ReceiveState.WAIT_END_2:
            if byte == self.END_BYTE_2:
                # 完整数据包接收完成
                rospy.logdebug("收到结束字节2，数据包完整")
                self.handle_complete_packet()
            else:
                rospy.logwarn("结束字节2错误，重新开始")
                self.reset_state_machine()
                if byte == self.START_BYTE_1:
                    self.rx_state = ReceiveState.WAIT_START_2
            
            # 无论成功还是失败，都重置状态机准备接收下一个包
            self.reset_state_machine()
        
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
