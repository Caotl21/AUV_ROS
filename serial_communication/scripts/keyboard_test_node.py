#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node('keyboard_input_node')
    pub = rospy.Publisher('servo_serial', String, queue_size=10)
    
    rospy.loginfo("键盘输入节点已启动，请输入角度数据（十进制）：")
    
    try:
        while not rospy.is_shutdown():
            data = input("输入角度数据 (输入 'quit' 退出): ").strip()
            
            if data.lower() in ['quit', 'exit', 'q']:
                rospy.loginfo("退出程序")
                break
                
            if data:
                msg = String(data)
                pub.publish(msg)
                rospy.loginfo(f"已发布: {data}")
                
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("程序被用户中断")

if __name__ == '__main__':
    main()
