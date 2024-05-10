#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation as R
from robot_move_server.srv import RobotMoveServerOrderRequest,RobotMoveServerOrderResponse,RobotMoveServerOrder


class relative_move:
    # 类初始化
    def __init__(self):
        self.current_posx=0
        self.current_posy=0
        self.current_roll=0
        self.current_pitch=0
        self.current_yaw=0
        self.current_velx=0
        self.current_vely=0
        self.current_velyaw=0
        self.current_time=0
        self.last_time=0
        self.Vel_c = rospy.Publisher("/husky_velocity_controller/cmd_vel",Twist,queue_size=10)
        self.Get_pose = rospy.Subscriber("/husky_velocity_controller/odom",Odometry,self.Get_poseCB,queue_size=10)
        self.robot_relative_move = rospy.Service("relative_move",RobotMoveServerOrder,self.relative_moveCB)
        # rospy.Service(serviceName, AddTwoInts, callback)
        # serviceName 参数为服务名称，是一个 uri 地址。
        # AddTwoInts 参数是服务需要的数据类型，这里使用的是 ros 提供的数据类型。
        # callback 参数为服务请求的回调。

    def Get_poseCB(self,Odom:Odometry):
        # rospy.loginfo("I heard:%s",Odom.data)
        self.current_posx = Odom.pose.pose.position.x
        self.current_posy = Odom.pose.pose.position.y
        quaternion = [Odom.pose.pose.orientation.x,Odom.pose.pose.orientation.y,Odom.pose.pose.orientation.z,Odom.pose.pose.orientation.w]
        r = R.from_quat(quaternion)
        euler = r.as_euler('xyz', degrees=True)
        self.current_roll=euler[0]/180*3.1415926
        self.current_pitch=euler[1]/180*3.1415926
        self.current_yaw=euler[2]/180*3.1415926
        self.current_velx = Odom.twist.twist.linear.x
        self.current_velyaw = Odom.twist.twist.angular.z
        rospy.loginfo("current_yaw:%.2f",self.current_yaw)
        # rospy.loginfo("robot_x:%.2f,robot_y:%.2f,robot_vx:%.2f,robot_rz:%.2f",self.current_posx,self.current_posy,vx,rz)
        # 俯仰角(Pitch)、偏航角(Yaw)和滚转角(Roll)
    
    def relative_moveCB(self,req):
        res=RobotMoveServerOrderResponse()
        if req.mode == "x":
            self.C_posx(req.value)
        elif req.mode == "yaw":
            self.C_yaw(req.value)
        elif req.mode == "y":
            self.C_posy(req.value)
        else:
            rospy.loginfo("mode 输入错误!")
            res.result = "fail"
            return res
        res.result = "success"
        return res

    def C_posx(self,C_pos):
        vel_msg = Twist()
        goal_pos= C_pos
        err = goal_pos
        self.last_time = rospy.Time.now()
        rate = rospy.Rate(10)
        while abs(goal_pos)>0.001:
            self.current_time = rospy.Time.now()
            delta_t = (self.current_time-self.last_time).to_sec()
            self.last_time = self.current_time
            goal_pos-=self.current_velx*delta_t
            rospy.loginfo("goal_pos:%.2f",goal_pos)
            vel_msg.linear.x = goal_pos # 相当于P控制
            self.Vel_c.publish(vel_msg)
            rate.sleep()
        vel_msg.linear.x = 0 # 速度清0
        self.Vel_c.publish(vel_msg)
    
    def C_yaw(self,C_pos):
        vel_msg = Twist()#定义1个速度变量，用于指定要控制的机器人速度
        goal_pos= C_pos
        err = goal_pos
        self.last_time = rospy.Time.now()
        rate = rospy.Rate(10)
        while abs(goal_pos)>0.001:
            self.current_time = rospy.Time.now()
            delta_t = (self.current_time-self.last_time).to_sec()
            self.last_time = self.current_time
            goal_pos-=self.current_velyaw*delta_t
            if goal_pos>3.14:
                goal_pos-=6.28
            elif(goal_pos <-3.14):
                goal_pos+=6.28
            vel_msg.angular.z = goal_pos
            self.Vel_c.publish(vel_msg)
            rate.sleep()
        vel_msg.angular.z = 0
        self.Vel_c.publish(vel_msg)
        
    def C_posy(self,C_pos):
        if C_pos<0:
            self.C_yaw(0.7854)
            self.C_posx(C_pos*(2 ** 0.5))
            self.C_yaw(-0.7854)
            self.C_posx(-C_pos)
        elif C_pos>0:
            self.C_yaw(-0.7854)
            self.C_posx(-C_pos*(2 ** 0.5))
            self.C_yaw(0.7854)
            self.C_posx(C_pos)


if __name__ == "__main__":
    # 2.初始化 ROS 节点
    rospy.init_node("robot_relative_move")
    # 3.创建服务对象
    relative_move = relative_move()
    # 5.spin 函数
    rospy.spin()