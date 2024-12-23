#!/usr/bin/env python
# -*- coding: utf-8 -*-

### 本程式為影像處理迴圈範例程式, 用於從Tello的影像中進行AprilTag偵測, 並將Tag的中心點顯示出來 ###
### 本程式已建立一rospy.Publisher, 會將Float64MultiArray發布至topic "/target_ap", ###
### 在第148行, 註解解除後, 會將AprilTag的中心點座標 center[0], center[1] 以及 AprilTag的面積w*h(透過左上右上, 左上右下點計算的 寬及高)發佈出去
### 此程式為適合用於pip install安裝AprilTag使用, 如果是透過github clone AprilTag (https://github.com/AprilRobotics/apriltag)下來使用, 請使用test_h264_sub_apv2.py ###

import rospy                                  # 導入套件: rospy
from sensor_msgs.msg import CompressedImage   # 導入 sensor_msgs 裡的 CompressedImage
from std_msgs.msg import Float64MultiArray    # 導入 std_msgs 裡的 Float64MultiArray
import av                                     # 導入套件: av
import apriltag                               # 導入套件: apriltag
import cv2                                    # 導入套件: cv2
import math                                   # 導入套件: math
import numpy as np                            # 導入套件: numpy 並命名成 np
import threading                              # 導入套件: threading
import traceback                              # 導入套件: traceback
import time                                   # 導入套件: time

### class StandaloneVideoStream 
### 用於將訂閱/tello/image_raw/h264所獲得的 CompressedImage 進行處理, 得到stream
### 再從stream中取得影像
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

# 建立 stream
stream = StandaloneVideoStream()

# 定義 function callback, 訂閱 "/tello/image_raw/h264", 用於處理接收到的compressedImg, (msg)請保留, msg即為收到的用於處理接收到的compressedImg()
def callback(msg):
    #rospy.loginfo('frame: %d bytes' % len(msg.data))
    stream.add_frame(msg.data)

# main function
def main():

    # fourcc: video的編碼格式, 如 XVID, MP4V 等等...
    fourcc = cv2.VideoWriter_fourcc('X', 'V', 'I', 'D')
    
    # out: 建立 VideoWriter, video名稱為 test.avi, 寫入格式為 'X',"V",'I','D', FPS 為 20.0, video解析度為 (影像寬, 影像高)
    out = cv2.VideoWriter('follow_demo.avi', fourcc, 20.0, (960, 720))
    
    # 建立 aprilTag Detector
    detector = apriltag.Detector()
    
    # 告訴ros此程式為node, node名稱為 'h264_listener'
    rospy.init_node('h264_listener')
    
    # 定義 rospy.Subscriber, 會訂閱 topic: '/tello/image_raw/h264' , CompressedImage為 topic: '/tello/image_raw/h264' 所需要的訊息格式
    # callback 為 接收訊息與處理function的名字, 可以自行定義, 但需一致
    rospy.Subscriber("/tello/image_raw/h264", CompressedImage, callback)
    
    #建立變數 point_pub, 建立rospy.Publisher, 會將 Float64MultiArray 訊息發布至 topic: /target_ap 上. queue_size表示暫存訊息的queue大小，當發布的訊息量超過queue_size時，就會開始剔除舊訊息
    point_pub = rospy.Publisher("/target_ap", Float64MultiArray, queue_size = 10)
    
    # 透過 套件 av 來開啟stream
    container = av.open(stream)
    
    # 在視窗上顯示 log: 'main: opened'
    rospy.loginfo('main: opened')
    
    # 由於 stream 會加入 一開始執行時就接收到的compressedImg, 因此會有一些過時的 compressedImg 需要拋棄
    # 這裡先拋棄 300 張
    frame_skip = 300
    
    # 對container進行解碼, 獲取已被加入至stream中的影像
    # 為 for loop
    # 當container解碼後發現沒有影像將會終止迴圈, 也代表 此程式會結束執行
    for frame in container.decode(video=0):
        
        # 當 frame_skip > 0, 則透過 continue 跳過該次影像處理迴圈, 並 frame_skip -1 
        if 0 < frame_skip:
          frame_skip -= 1
          continue
        
        # 計算影像處理迴圈一次所需要花費的時間, 先建立 start_time 作為起始
        start_time = time.time()  
        
        # 透過 np.array 將 frame 先轉成np格式, 再透過cvtColor 將RGB格式轉換成BGR格式
        image = cv2.cvtColor(np.array(
            frame.to_image()), cv2.COLOR_RGB2BGR)
        
        # 使用cvtColor 將 BGR 格式轉換成 Gray 格式
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 使用 aprilTag Detector 進行 aprilTag偵測, detector需要gray img 進行偵測, 這裡將轉換成 gray格式的 gray_image 作為輸入
        # result 為偵測結果, 為1 list []
        # 當list 長度為 0, 表示沒有偵測到 aprilTag
        # 當list 長度不為0, 表示有偵測到 aprilTag
        # detector可以同時偵測多個 aprilTag    
        result = detector.detect(gray_image)
        
        # 複製一份 原始影像image 作為 show_image
        show_image = image.copy()
        
        # 當 沒有偵測到 aprilTag, 在視窗上顯示 "not found"
        if len(result) == 0:
          print("not found")
        # 當 有偵測到 aprilTag, 取出其中心點 center 與四個角落 corners 座標
        else:
          # result 為 1 list, 故透過 [0] 取出第一個 aprilTag
          center = result[0].center
          corner = result[0].corners
          
          # 透過左下角與右下角計算寬 w
          w = math.sqrt( (corner[0][0] - corner[1][0])**2 + (corner[0][1] - corner[1][1])**2 )
          
          # 透過右下角與右上角計算寬 h
          h = math.sqrt( (corner[2][0] - corner[1][0])**2 + (corner[2][1] - corner[1][1])**2 )
          
          # 顯示 計算出來的 寬, 高, 以及面積
          print(w, h, w*h)
          
          #註解解除後, 會將AprilTag的中心點座標 center[0], center[1] 以及 AprilTag的面積w*h(透過左上右上, 左上右下點計算的 寬及高)發佈出去
          #point_pub.publish(Float64MultiArray(data = [center[0], center[1], w*h ]))
          
          # 透過circle 將一圓形畫在show_image上,
          # (int(center[0]),int(center[1])) 為圓心, 需為 int
          # 5的位置為 半徑
          # (0,0,255)的位置為顏色
          # -1的位置為圓形線條寬度, -1 為填滿, 其餘則為 > 0 為線條寬度
          # 這裡將 aprilTag 的中心點畫在 show_image 上
          cv2.circle(show_image, (int(center[0]),int(center[1])) , 5, (0,0,255), -1)
          
          # 透過polylines 將一多邊形畫在show_image上,
          # [np.int32(corner)] 為多邊形的點list
          # True 的位置 表示 是否要封閉, True為封閉, False為開放
          # (0,255,0)的位置為顏色
          # 2 的位置表示 線條寬度
          # cv2.LINE_AA 表示線條表示種類
          cv2.polylines(show_image,[np.int32(corner)], True, (0,255,0), 2, cv2.LINE_AA)
        
        # 將show_image 寫入到video: out中 
        out.write(show_image)
        
        # 顯示 影像, 視窗名稱為result, 欲顯示的cv_img為 show_image
        cv2.imshow('result', show_image)
        
        # 設定視窗刷新頻率
        cv2.waitKey(1)
        
        # 計算 stream中的FPS
        if frame.time_base < 1.0/60:
          time_base = 1.0/60
        else:
          time_base = frame.time_base
        # 根據FPS, 以及當前時間 time.time() 減掉起始時間 start_time後 除以FPS得到 新的需跳過影像張數 frame_skip
        frame_skip = int((time.time() - start_time)/time_base)

# main
if __name__ == '__main__':

    # try catch
    try:
        # 執行 main fuction, 啟動影像處理迴圈
        main()
    # 例外處理: BaseException, 有發生會透過 traceback 顯示例外錯誤
    except BaseException:
        traceback.print_exc()
    
    # 最終處理: 關掉 stream, 並將cv2所產生的影像視窗都關掉
    finally:
        stream.close()
        cv2.destroyAllWindows()
