#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import simple_tello
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from behave import *
from time import sleep

t1 = simple_tello.Tello_drone()

class bt_mission:
    isContinue = True
    center = (480, 200)
    dx = -1
    dy = -1
    
    # 新增狀態變數
    has_passed_first_frame = False
    waiting_complete = False
    has_turned = False
    has_detected_tag = False
    
    def __init__(self):
        # 修改後的行為樹結構
        self.tree = (
            # 第一階段：過第一個框
            (self.isNotPassedFirstFrame >> 
                ((self.isNotDataReceived >> self.doHover)
                | (self.isReceivePass >> self.doAddSp >> self.doMarkFirstFramePassed)
                | (self.doComputeData >> (self.isForward >> self.doForward) | (self.doCorrection))))
            
            # 第二階段：等待5秒
            | (self.isWaitingPhase >> self.doWait)
            
            # 第三階段：左轉
            | (self.isTurningPhase >> self.doTurn)
            
            # 第四階段：等待看到 AprilTag
            | (self.isWaitingForTag >> self.doWaitForTag)
            
            # 第五階段：過第二個框
            | ((self.isNotDataReceived >> self.doHover)
                | (self.isReceivePass >> self.doAddSp >> self.doBtStop)
                | (self.doComputeData >> (self.isForward >> self.doForward) | (self.doCorrection)))
        )
        
    # 新增條件節點：檢查是否尚未通過第一個框
    @condition
    def isNotPassedFirstFrame(self):
        return not self.has_passed_first_frame
        
    # 新增動作節點：標記已通過第一個框
    @action
    def doMarkFirstFramePassed(self):
        bt_mission.has_passed_first_frame = True
        
    # 新增條件節點：檢查是否處於等待階段
    @condition
    def isWaitingPhase(self):
        return self.has_passed_first_frame and not self.waiting_complete
        
    # 新增動作節點：等待5秒
    @action 
    def doWait(self):
        msg = Twist()
        t1.controler.move(msg, 5.0)  # 懸停5秒
        bt_mission.waiting_complete = True
        
    # 新增條件節點：檢查是否處於轉向階段
    @condition
    def isTurningPhase(self):
        return self.waiting_complete and not self.has_turned
        
    # 新增動作節點：執行左轉
    @action
    def doTurn(self):
        msg = Twist()
        msg.angular.z = 0.5  # 正值為左轉
        t1.controler.move(msg, 2.0)  # 轉動2秒
        bt_mission.has_turned = True
        
    # 新增條件節點：檢查是否等待看到 AprilTag
    @condition
    def isWaitingForTag(self):
        return self.has_turned and not self.has_detected_tag
        
    # 新增動作節點：等待直到看到 AprilTag
    @action
    def doWaitForTag(self):
        if t1.state.target_x != -1 and t1.state.target_y != -1:
            bt_mission.has_detected_tag = True
            
    # 保留原有的其他節點...
    @condition
    def isNotDataReceived(self):
        return t1.state.target_x == -1 and t1.state.target_y == -1

    @action
    def doHover(self):
        msg = Twist()
        t1.controler.move(msg, 0.5)

    @condition
    def isReceivePass(self):
        return t1.state.canPass == 1

    @action
    def doAddSp(self):
        msg = Twist()
        msg.linear.y = 0.4
        t1.controler.move(msg, 3)
        msg = Twist()
        msg.linear.y = 0.5
        t1.controler.move(msg, 3)

    @action
    def doBtStop(self):
        bt_mission.isContinue = False

    @action
    def doComputeData(self):
        bt_mission.dx = t1.state.target_x - bt_mission.center[0]
        bt_mission.dy = t1.state.target_y - bt_mission.center[1]
     
    @condition
    def isForward(self):
        return abs(bt_mission.dx) < 30 and abs(bt_mission.dy) < 30
    
    @action
    def doForward(self):
        msg = Twist()
        msg.linear.y = 0.2
        t1.controler.move(msg, 0.5)
    
    @action
    def doCorrection(self):
        msg = Twist()
        if bt_mission.dx != 0:
          msg.linear.x = bt_mission.dx / abs(bt_mission.dx) * 0.1
        if bt_mission.dy != 0:
          msg.linear.z = -bt_mission.dy / abs(bt_mission.dy) * 0.2
        t1.controler.move(msg, 0.5)

    def run(self):
        while True:
            if bt_mission.isContinue == False:
                break
            bb = self.tree.blackboard(1)
            state = bb.tick()
            print("state = %s\n" % state)
            while state == RUNNING:
                state = bb.tick()
                print("state = %s\n" % state)
            assert state == SUCCESS or state == FAILURE