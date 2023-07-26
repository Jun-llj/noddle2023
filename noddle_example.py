#!/usr/bin/env python3
# coding: UTF-8
# date: 2023/7/19

import rospy
from noddle_navigator import Navigator  # 导航模块
from noddle_recognizer import Recognizer  # 语音识别模块和分析模块
from base_controller import Base  # 底盘运动模块
from std_msgs.msg import String  # std_msgs中包含消息类型string，发布的消息类型为String，从String.data中可获得信息，


import os

LOCATION = {  # 储存导航路径点
    '厨房': [[-1.658400, -0.046712, 0.000000], [0.000000, 0.000000, -0.986665, 0.162761]],
    '客厅': [[3.859466, -2.201285, 0.000000], [0.000000, 0.000000, -0.247601, -0.968862]],
    '卧室': [[-1.658400, -0.046712, 0.000000], [0.000000, 0.000000, -0.986665, 0.162761]],
    '餐厅': [[3.859466, -2.201285, 0.000000], [0.000000, 0.000000, -0.247601, -0.968862]],
}

class Controller:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)  # 初始化ros节点，告诉rospy你的节点名字，叫name，
        rospy.Subscriber('/start_signal', String, self.control)  # 创建订阅者订阅recognizer发出的地点作为启动信号
        self.navigator = Navigator(LOCATION)  # 实例化导航模块
        self.recognizer = Recognizer()  # 实例化语音唤醒、识别和逻辑判断模块
        self.base = Base()  # 实例化移动底盘模块
        self.recognizer.get_cmd()  # 获取一次语音命令

    def control(self, place):
        """订阅start signal的回调函数,传入的place是String类型消息 .data可以获取传来的信息,即目标房间"""
        self.goal = place.data  # 存入目标房间名字
        self.navigator.goto(place.data)
        os.system('espeak -vzh ' + '请跟随我.')
        self.navigator.goto(self.goal)  # 导航模块调用goto方法,传入去的地点名字符串即可导航区指定地点


if __name__ == '__main__':
    try:
        Controller('noddle_example')  # 实例化Controller,参数为初始化ros节点使用到的名字
        rospy.spin()  # 保持监听订阅者订阅的话题，直到节点已经关闭
    except rospy.ROSInterruptException:
        pass