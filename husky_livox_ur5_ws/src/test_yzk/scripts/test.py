#! /usr/bin/env python

"""
    Python 版 HelloWorld

"""
import rospy

if __name__ == "__main__":
    rospy.init_node("Hello")
    rospy.loginfo("Hello World!!!!")
