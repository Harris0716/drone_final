#!/usr/bin/env python
# -*- coding: utf-8 -*-

### 建立class TelloState, TelloController 以及 Tello_drone ###
### 用於儲存 Tello 資訊、透過ros發布訊息,執行服務來控制 Tello 以及處理相關 ros 訂閱部分  ###

import rospy # 導入套件: rospy
import time  # 導入套件: time

from geometry_msgs.msg import Twist                       # 導入 geometry_msgs 裡的 Twist 訊息格式 
from std_msgs.msg import Empty, UInt8, Float64MultiArray  # 導入 std_msgs 裡的 Empty, UInt8, Float64MultiArray 訊息格式
from time import sleep                                    # 導入 time 裡的 sleep 
from tello_driver.msg import TelloStatus                  # 導入 tello_driver 裡的 TelloStatus 訊息格式

### class TelloState
### 用於儲存從topic訂閱所獲得的資訊
### 初始值設定可以根據需求調整
class TelloState:
    
    # init function, 進行 self變數 初始化
    def __init__(self):
    
        self.height = 0.0                 # 紅外線測高
        self.temperature_height_m = 0.0   # 氣壓計測高
        self.battery = 100.0              # 電池電量
        self.is_flying = False            # 是否正在飛行
        self.fly_mode = 999               # 目前的飛行模式
        self.target_x = -1                # 藉由訂閱topic 所獲得的框中心點 x 
        self.target_y = -1                # 藉由訂閱topic 所獲得的框中心點 y    
        self.canPass = -1                 # 藉由訂閱topic 所獲得是否mask面積超過門檻值, -1 表示尚未收到, 0 表示尚未超過門檻值, 1 表示超過門檻值

### class TelloController
### 用於建立與rospy publisher, serviceProxy有關之function
### 需要進行發布或是請求服務可以寫成function的,避免主程式過於雜亂
class TelloController:

    # 定義function: move, 用於控制 Tello 移動, twist為已經輸入linear與angular, limitTime為持續發送時間
    def move(self, twist, limitTime):
        limitTime = limitTime * 1000 # 秒轉成毫秒
        startTime = int(round(time.time()*1000)) # 透過time.time獲取現在時間作為起始時間, 並將秒轉成毫秒
        rate = rospy.Rate(10) # 定義變數 rate, 10 表示 ros 會盡可能地讓下列 while迴圈 1秒內執行 10 次
        
        #建立變數pub_move, 建立rospy.Publisher, 會將 Twist訊息 發布至 topic: /tello/cmd_vel 上. queue_size表示暫存訊息的queue大小，當發布的訊息量超過queue_size時，就會開始剔除舊訊息
        pub_move = rospy.Publisher("/tello/cmd_vel", Twist, queue_size = 10)

        # 執行 while迴圈, 終止條件為 roscore停止時, 或是被break
        while not rospy.is_shutdown():
          
          #透過 get_num_connections 取得 目前topic已被連接數量, 大於0表示我們的程式與topic之間的連線已建立,然後再發布訊息. 沒有確認數量就進行發送有可能會導致空發訊息的狀況發生,導致Tello沒有移動
          connections = pub_move.get_num_connections()
          if connections > 0:
            endTime = int(round(time.time()*1000)) # 建立變數endTime, 用作當前時間, 並將秒轉成毫秒
            if endTime - startTime < limitTime:    # 當前時間與起始時間相減得到經過時間, 當還在持續發送時間內則繼續發送
              pub_move.publish(twist)              # 將 twist 透過 pub_move 發佈出去 
            else:                                  # 當前時間與起始時間相減得到經過時間, 當超過持續發送時間則終止while迴圈
              break
            rate.sleep()                           # 因rate設置為10, 因此會嘗試讓本次while迴圈到下次執行迴圈之間達到10 HZ
    
    # 定義function: emergency: 用途為緊急停止無人機的懸翼旋轉, 在空中使用會使無人機掉下來
    def emergency(self):
        rate = rospy.Rate(10) # 定義變數 rate, 10 表示 ros 會盡可能地讓下列 while迴圈 1秒內執行 10 次
        
        #建立變數puber, 建立rospy.Publisher, 會將 Empty訊息發布至 topic: /tello/emergency 上. queue_size表示暫存訊息的queue大小，當發布的訊息量超過queue_size時，就會開始剔除舊訊息
        puber = rospy.Publisher("/tello/emergency", Empty, queue_size=10) 
        
        # 執行 while迴圈, 終止條件為 roscore停止時, 或是被break
        while not rospy.is_shutdown():
         
          #透過 get_num_connections 取得 目前topic已被連接數量, 大於0表示我們的程式與topic之間的連線已建立,然後再發布訊息. 沒有確認數量就進行發送有可能會導致空發訊息的狀況發生,導致Tello沒有執行相應行為
          cons = puber.get_num_connections()
          if cons > 0:
            puber.publish(Empty()) # 將 Empty訊息 透過 puber 發佈出去
            rate.sleep()           # 因rate設置為10, 因此會嘗試讓本次while迴圈到下次執行迴圈之間達到10 HZ
            break                  # 中斷迴圈
        
    #定義function: takeoff: 用途為讓Tello起飛
    def takeoff(self):
        rate = rospy.Rate(10) # 定義變數 rate, 10 表示 ros 會盡可能地讓下列 while迴圈 1秒內執行 10 次

        #建立變數puber, 建立rospy.Publisher, 會將 Empty訊息發布至 topic: /tello/takeoff 上. queue_size表示暫存訊息的queue大小，當發布的訊息量超過queue_size時，就會開始剔除舊訊息
        puber = rospy.Publisher("/tello/takeoff", Empty, queue_size=10) 
        
        # 執行 while迴圈, 終止條件為 roscore停止時, 或是被break
        while not rospy.is_shutdown():
          
          #透過 get_num_connections 取得 目前topic已被連接數量, 大於0表示我們的程式與topic之間的連線已建立,然後再發布訊息. 沒有確認數量就進行發送有可能會導致空發訊息的狀況發生,導致Tello沒有執行相應行為
          cons = puber.get_num_connections()
          if cons > 0:
            puber.publish(Empty()) # 將 Empty訊息 透過 puber 發佈出去
            rate.sleep()           # 因rate設置為10, 因此會嘗試讓本次while迴圈到下次執行迴圈之間達到10 HZ
            break                  # 中斷迴圈
    
    #定義function: land: 用途為讓Tello降落     
    def land(self):
        rate = rospy.Rate(10) # 定義變數 rate, 10 表示 ros 會盡可能地讓下列 while迴圈 1秒內執行 10 次
                
        #建立變數puber, 建立rospy.Publisher, 會將 Empty訊息發布至 topic: /tello/land 上. queue_size表示暫存訊息的queue大小，當發布的訊息量超過queue_size時，就會開始剔除舊訊息
        puber = rospy.Publisher("/tello/land", Empty, queue_size=10) 
        
        # 執行 while迴圈, 終止條件為 roscore停止時, 或是被break
        while not rospy.is_shutdown():
          
          #透過 get_num_connections 取得 目前topic已被連接數量, 大於0表示我們的程式與topic之間的連線已建立,然後再發布訊息. 沒有確認數量就進行發送有可能會導致空發訊息的狀況發生,導致Tello沒有執行相應行為
          cons = puber.get_num_connections()
          if cons > 0:
            puber.publish(Empty()) # 將 Empty訊息 透過 puber 發佈出去
            rate.sleep()           # 因rate設置為10, 因此會嘗試讓本次while迴圈到下次執行迴圈之間達到10 HZ
            break                  # 中斷迴圈

    #定義function: flip: 用途為讓Tello翻滾, i 為整數, 表示翻滾方向
    def flip(self, i):
        rate = rospy.Rate(10) # 定義變數 rate, 10 表示 ros 會盡可能地讓下列 while迴圈 1秒內執行 10 次
        
        #建立變數puber, 建立rospy.Publisher, 會將 UInt8訊息發布至 topic: /tello/flip 上. queue_size表示暫存訊息的queue大小，當發布的訊息量超過queue_size時，就會開始剔除舊訊息
        puber = rospy.Publisher("/tello/flip", UInt8, queue_size=10) 
        
        # 執行 while迴圈, 終止條件為 roscore停止時, 或是被break
        while not rospy.is_shutdown():
          
          #透過 get_num_connections 取得 目前topic已被連接數量, 大於0表示我們的程式與topic之間的連線已建立,然後再發布訊息. 沒有確認數量就進行發送有可能會導致空發訊息的狀況發生,導致Tello沒有執行相應行為
          cons = puber.get_num_connections()
          if cons > 0:
            puber.publish(UInt8(data=i)) # 將 UInt8訊息 透過 puber 發佈出去, 翻滾方向透過 data=i 的方式做參數輸入
            rate.sleep()                 # 因rate設置為10, 因此會嘗試讓本次while迴圈到下次執行迴圈之間達到10 HZ
            break                        # 中斷迴圈
 
### class Tello_drone()
### 用於建立物件 Tello_drone
### 會在init中將 相關屬性以及上面的兩個class TelloState, TelloController 建立起來
### 在_sensor()中, 會去訂閱相關的topic, 取得Tello的飛行資訊   
class Tello_drone():
    def __init__(self):
    
        self.state = TelloState()          #Tello 的飛行資訊儲存物件
        self.controler = TelloController() #Tello 的相關publisher, serviceProxy執行function物件
        self._sensor()                     #執行_sensor()來建立rospy.Subscriber

    # 用於建立rospy.Subscriber的地方
    def _sensor(self):
        _ts_sub = rospy.Subscriber("/tello/status", TelloStatus, self._ts_cb, queue_size = 10)        # 建立變數 _ts_sub, 用rospy.Subscriber向topic "/tello/status" 進行訂閱, 該topic 所需的訊息格式為TelloStatus, 負責接收訊息的function名為self._ts_cb, queue_size為暫存訊息的queue大小, 10表示最多一次可以接收10筆
        _tp_sub = rospy.Subscriber("/target_point", Float64MultiArray, self._tp_cb, queue_size = 10)  # 建立變數 _tp_sub, 用rospy.Subscriber向topic "/target_point" 進行訂閱, 該topic 所需的訊息格式為Float64MultiArray, 負責接收訊息的function名為self._tp_cb, queue_size為暫存訊息的queue大小, 10表示最多一次可以接收10筆, 該topic在 test_h264_sub.py進行訊息發佈, 此處為接收
    
    # _ts_sub 的 接收訊息function, 用於接收來自/tello/status 的訊息, 其表示Tello目前的飛行資訊與感測器資訊, 數量很多, 可以自行決定要使用哪些資訊
    def _ts_cb(self, data):
    
        # 使用 self.state裡的變數來儲存
        self.state.height = data.height_m                            # 紅外線測高 
        self.state.temperature_height_m = data.temperature_height_m  # 氣壓計測高
        self.state.battery = data.battery_percentage                 # 電池電量
        self.state.is_flying = data.is_flying                        # 是否正在飛行
        self.state.fly_mode = data.fly_mode                          # 目前的飛行模式
    
    # _tp_sub 的 接收訊息function, 用於接收來自/target_point 的訊息, 其表示目前抓到的HSV mask 中心點 與mask面積是否超過門檻值
    def _tp_cb(self, data):
        
        # 使用 self.state裡的變數來儲存
        self.state.target_x = data.data[0]  # 藉由訂閱topic: /target_point 所獲得的框中心點 x
        self.state.target_y = data.data[1]  # 藉由訂閱topic: /target_point 所獲得的框中心點 y
        self.state.canPass = data.data[2]   # 藉由訂閱topic: /target_point 所獲得是否mask面積超過門檻值
