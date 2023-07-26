#!/usr/bin/env python
# coding: UTF-8
# date: 2023/7/21

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
import os

LOCATION = {  # 模糊词
    '厨房': ['厨房'],
    '客厅': ['客厅'],
    '卧室': ['卧室'],
    '餐厅': ['餐厅'],
}

class Recognizer:
    def __init__(self):
        self.wakeup = rospy.Publisher('/if_awake_flag', Int8, queue_size=1)  # 实例化语音唤醒
        self.start_signal = rospy.Publisher('/start_signal', String, queue_size=10)
        self.cmd = None
        self.location = LOCATION
        self.goal = ''
        self.status = 0  # 是否已得到想要的命令
        self.key = 1
        rospy.Subscriber('/voice_words', String, self.voice_words_talkback)  # 接收被识别出的语音

    def voice_words_talkback(self, msg):
        if self.key == 1:
            print("\n读入的信息为: " + msg.data)
            self.cmd = msg.data
            self.judge()

    def judge(self):
        if self.status == 0:  # 未得到想要的命令

            response = self.analyze()

            if response == '您是否需要我':
                os.system('espeak -vzh ' + "请说命令.")
                self.get_cmd()
            else:
                self.status = 1  # 已得到想要的命令
                print(response)
                os.system('espeak -vzh ' + response)
                os.system('espeak -vzh ' + '请说是或否.')
                print('请说是或否.')
                self.get_cmd()

        elif ('是' in self.cmd) and (self.status == 1):

            os.system('espeak -vzh ' + '好的.')
            print('好的.')
            self.start_signal.publish(self.goal)
            self.key = 0
            self.status = 0
            self.goal = ''


        elif ('否' in self.cmd) and (self.status == 1):
            os.system('espeak -vzh ' + "请再说一次命令.")
            print("请再说一次命令. ")
            self.status = 0
            self.goal = ''
            self.get_cmd()

        else:
            # os.system('espeak -vzh ' + '请说是或否.')
            print('请说是或否.')
            self.get_cmd()

    # def processed_cmd(self, cmd):
    #     cmd = cmd.lower()
    #     for i in " ,.;?":
    #         cmd = cmd.replace(i, ' ')
    #     return cmd

    def get_cmd(self):
        """获取一次命令"""
        os.system('espeak -vzh ' + '请您说话.')
        self.wakeup.publish(1)

    def analyze(self):
        response = '您是否需要我'
        for (key, val) in self.location.items():
            for word in val:
                if word in self.cmd:
                    self.goal = key
                    response = response + '去' + key + '?'
                    break
        return response


if __name__ == '__main__':
    try:
        rospy.init_node('noddle_recognition')
        Recognizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
