#! /usr/bin/env python
"""
    需求: 实现基本的话题通信，一方发布数据，一方接收数据，
         实现的关键点:
         1.发送方
         2.接收方
         3.数据(此处为普通文本)


    消息订阅方:
        订阅话题并打印接收到的消息

    实现流程:
        1.导包 
        2.初始化 ROS 节点:命名(唯一)
        3.实例化 订阅者 对象
        4.处理订阅的消息(回调函数)
        5.设置循环调用回调函数



"""
#1.导包 
import rospy
from nav_msgs.msg import Odometry

def Get_poseCB(Odom:Odometry):
    # rospy.loginfo("I heard:%s",Odom.data)
    x = Odom.pose.pose.position.x
    y = Odom.pose.pose.position.y
    z = Odom.pose.pose.position.z
  
    vx = Odom.twist.twist.linear.x
    rz = Odom.twist.twist.angular.z
    rospy.loginfo("robot_x:%.2f,robot_y:%.2f,robot_vx:%.2f,robot_rz:%.2f",x,y,vx,rz)

if __name__ == "__main__":
    #2.初始化 ROS 节点:命名(唯一)
    rospy.init_node("robot_pose")
    #3.实例化 订阅者 对象
    sub = rospy.Subscriber("/husky_velocity_controller/odom",Odometry,Get_poseCB,queue_size=10)
    #4.处理订阅的消息(回调函数)
    #5.设置循环调用回调函数
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()
