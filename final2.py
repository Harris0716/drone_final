#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import simple_tello
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64MultiArray
import av
import apriltag
import cv2
import math
import numpy as np
import threading
import traceback
import time
from behave import *
from time import sleep

# 建立無人機物件
t1 = simple_tello.Tello_drone()

# 影像串流類別
class StandaloneVideoStream(object):
    def __init__(self):
        self.cond = threading.Condition()
        self.queue = []
        self.closed = False

    def read(self, size):
        self.cond.acquire()
        try:
            if len(self.queue) == 0 and not self.closed:
                self.cond.wait(2.0)
            data = bytes()
            while 0 < len(self.queue) and len(data) + len(self.queue[0]) < size:
                data = data + self.queue[0]
                del self.queue[0]
        finally:
            self.cond.release()
        return data

    def seek(self, offset, whence):
        return -1

    def close(self):
        self.cond.acquire()
        self.queue = []
        self.closed = True
        self.cond.notifyAll()
        self.cond.release()

    def add_frame(self, buf):
        self.cond.acquire()
        self.queue.append(buf)
        self.cond.notifyAll()
        self.cond.release()

# 建立串流物件
stream = StandaloneVideoStream()

# 影像回調函數
def callback(msg):
    stream.add_frame(msg.data)

# 行為樹類別
class bt_mission:
    isContinue = True
    center = (480, 200)
    dx = -1
    dy = -1
    detection_started = False

    def __init__(self):
        self.tree = (
            (self.isNotDataReceived >> self.doHover)
            | (self.isReceivePass >> self.doAddSp >> self.doTurnLeft >> self.startDetection)
            | (self.doComputeData >> (self.isForward >> self.doForward) | (self.doCorrection)) 
        )

    @condition
    def isNotDataReceived(self):
        print("condition: isNotDataReceived")
        return t1.state.target_x == -1 and t1.state.target_y == -1

    @action
    def doHover(self):
        print("action: doHover")
        msg = Twist()
        t1.controler.move(msg, 0.5)

    @condition
    def isReceivePass(self):
        print("condition: isReceivePass")
        return t1.state.canPass == 1

    @action
    def doAddSp(self):
        print("action: doAddSp")
        msg = Twist()
        msg.linear.y = 0.4
        t1.controler.move(msg, 3)
      
        msg = Twist()
        msg.linear.y = 0.5
        t1.controler.move(msg, 3)

    @action
    def doTurnLeft(self):
        print("action: doTurnLeft")
        msg = Twist()
        msg.angular.z = 0.5  # 轉向速度
        t1.controler.move(msg, 0.9)  # 轉向時間改為 0.9 秒，實現約 90 度轉向

    @action
    def startDetection(self):
        print("action: startDetection")
        bt_mission.detection_started = True
        bt_mission.isContinue = False
        # 開始執行 AprilTag 偵測
        start_apriltag_detection()

    @action
    def doComputeData(self):
        print("action: doComputeData")
        bt_mission.dx = t1.state.target_x - bt_mission.center[0]
        bt_mission.dy = t1.state.target_y - bt_mission.center[1]
        print("doComputeData: ", bt_mission.dx, bt_mission.dy)
     
    @condition
    def isForward(self):
        print("condition: isForward")
        return abs(bt_mission.dx) < 30 and abs(bt_mission.dy) < 30
    
    @action
    def doForward(self):
        print("action: doForward")
        msg = Twist()
        msg.linear.y = 0.2
        t1.controler.move(msg, 0.5)
    
    @action
    def doCorrection(self):
        print("action: doCorrection")
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

def start_apriltag_detection():
    # AprilTag 偵測的主要功能
    fourcc = cv2.VideoWriter_fourcc('X', 'V', 'I', 'D')
    out = cv2.VideoWriter('follow_demo.avi', fourcc, 20.0, (960, 720))
    detector = apriltag.Detector()
    
    point_pub = rospy.Publisher("/target_ap", Float64MultiArray, queue_size=10)
    container = av.open(stream)
    
    frame_skip = 300
    
    for frame in container.decode(video=0):
        if 0 < frame_skip:
            frame_skip -= 1
            continue
        
        start_time = time.time()
        
        image = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        result = detector.detect(gray_image)
        show_image = image.copy()
        
        if len(result) == 0:
            print("not found")
        else:
            center = result[0].center
            corner = result[0].corners
            
            w = math.sqrt((corner[0][0] - corner[1][0])**2 + (corner[0][1] - corner[1][1])**2)
            h = math.sqrt((corner[2][0] - corner[1][0])**2 + (corner[2][1] - corner[1][1])**2)
            
            print(w, h, w*h)
            point_pub.publish(Float64MultiArray(data=[center[0], center[1], w*h]))
            
            cv2.circle(show_image, (int(center[0]), int(center[1])), 5, (0,0,255), -1)
            cv2.polylines(show_image, [np.int32(corner)], True, (0,255,0), 2, cv2.LINE_AA)
        
        out.write(show_image)
        cv2.imshow('result', show_image)
        cv2.waitKey(1)
        
        if frame.time_base < 1.0/60:
            time_base = 1.0/60
        else:
            time_base = frame.time_base
        frame_skip = int((time.time() - start_time)/time_base)

def main():
    while t1.state.is_flying == False:
        t1.controler.takeoff()
    
    while t1.state.fly_mode != 6:
        print("wait...")
    
    # 訂閱影像串流
    rospy.Subscriber("/tello/image_raw/h264", CompressedImage, callback)
    
    btCm_n = bt_mission()
    sleep(2)
    print("bt start...")
    btCm_n.run()
    
    while t1.state.fly_mode != 6:
        print("wait...")

if __name__ == "__main__":
    try:
        rospy.init_node('tello_control', anonymous=True)
        main()
    except BaseException:
        traceback.print_exc()
    finally:
        stream.close()
        cv2.destroyAllWindows()