#!/usr/bin/env python
# -*- coding: utf-8 -*-

### 影像處理迴圈範例程式 ###
### 該程式 會訂閱 tello的影像topic, 並以 StandaloneVideoStream 將收到的影像製作成stream ###
### 在main中, 透過對stream進行for loop取得cv img, 來進行HSV 過濾, 尋找輪廓與計算輪廓的中心點 ###
### 透過minAreaRect 得到一包覆輪廓mask的最小方形, 根據其寬與高來得到mask的面積 ###
### 根據 mask面積 / 畫面總面積 (960 * 720) 來計算 比例, 當比例超過門檻值表示已經接近框 1, 當比例未超過門檻值 0
### 最後透過rospy.Publisher, 將 輪廓的中心點, 比例是否超過門檻值 發送出去 ###

import rospy                                 # 導入套件: rospy
from sensor_msgs.msg import CompressedImage  # 導入 sensor_msgs 裡的 CompressedImage 訊息格式
from std_msgs.msg import Float64MultiArray   # 導入 std_msgs 裡的 Float64MultiArray 訊息格式
import av                                    # 導入套件: av
import cv2                                   # 導入套件: cv2
import numpy as np                           # 導入套件: numpy 並命名成 np 
import threading                             # 導入套件: threading
import traceback                             # 導入套件: traceback
import time                                  # 導入套件: time

### class StandaloneVideoStream
### 用於將訂閱 /tello/image_raw/h264 所獲得的 CompressedImage 進行處理, 得到stream
### 再從 stream 中取得影像
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

# 定義 function callback, 訂閱 "/tello/image_raw/h264", 用於處理接收到的 CompressedImage, (msg)請保留, msg 即為compressedImg()
def callback(msg):
    #rospy.loginfo('frame: %d bytes' % len(msg.data)) # 可以解開註解來觀看目前收到的bytes穩不穩定
    # 將接收到的CompressedImage加入到stream中
    stream.add_frame(msg.data)

# 定義function: find_Mask, 輸入為cv hsv_img(HSV格式), 用途為根據給定的HSV 上下界值域進行過濾 從影像中找出特定顏色的mask
def find_Mask(img):

  # 定義 HSV 值域的下界, 詳見投影片中 HSV 範圍圖中的左紅框
  lr0 = np.array([0, 70, 0])
  # 定義 HSV 值域的上界, 詳見投影片中 HSV 範圍圖中的左紅框
  ur0 = np.array([5, 255, 255])
  
  # 定義 HSV 值域的下界, 詳見投影片中 HSV 範圍圖中的右紅框
  lr1 = np.array([175, 70, 0])
  # 定義 HSV 值域的上界, 詳見投影片中 HSV 範圍圖中的右紅框
  ur1 = np.array([180, 255, 255])
  
  # 透過第一組左紅框的HSV值域進行影像過濾
  rm0 = cv2.inRange(img, lr0, ur0)
  
  # 透過第一組右紅框的HSV值域進行影像過濾
  rm1 = cv2.inRange(img, lr1, ur1)
  
  # 將第一組, 第二組紅框值域進行or運算, 將兩者結果合併
  rm = cv2.bitwise_or(rm0, rm1)
  return rm # 回傳結果

# main function
def main():
    
    # fourcc: video的編碼格式, 如 XVID, MP4V 等等...
    fourcc = cv2.VideoWriter_fourcc('X', 'V', 'I', 'D')
    # out: 建立 VideoWriter, video名稱為 test.avi, 寫入格式為 'X',"V",'I','D', FPS 為 20.0, video解析度為 (影像寬, 影像高)
    # 由於這邊會輸出左影像為原始影像, 右影像為視覺處理的影像, 因此在寬上面會是960*2 = 1920, 高720不變
    out = cv2.VideoWriter('test.avi', fourcc, 20.0, (1920, 720))
    
    # 告訴ros此程式為node, node名稱為 'h264_listener'
    rospy.init_node('h264_listener')
    
    # 定義 rospy.Subscriber, 會訂閱 topic: '/tello/image_raw/h264' , CompressedImage為 topic: '/tello/image_raw/h264' 所需要的訊息格式
    # callback 為 接收訊息與處理function的名字, 可以自行定義, 但需一致
    rospy.Subscriber("/tello/image_raw/h264", CompressedImage, callback)
    
    #建立變數 point_pub, 建立rospy.Publisher, 會將 Float64MultiArray 訊息發布至 topic: /target_point 上. queue_size表示暫存訊息的queue大小，當發布的訊息量超過queue_size時，就會開始剔除舊訊息
    point_pub = rospy.Publisher("/target_point", Float64MultiArray, queue_size = 10)
    
    # 透過 套件 av 來開啟stream
    container = av.open(stream)
    
    # 在視窗上顯示 log: 'main: opened'
    rospy.loginfo('main: opened')
    
    # 建立 bool 變數 start_detect, 當 start_detect 為 True 時, 會透過 point_pub 發送 Float64MultiArray 至 /target_point
    # 當 start_detect 為 False, 則不發送
    start_detect = True
    
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
            
        # 使用cvtColor 將 BGR 格式轉換成 HSV 格式
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 執行 function: findMask, 將 hsv_img 作為 參數輸入, red_mask為 HSV 過濾後的結果
        red_mask = find_Mask(hsv_img)
        
        # 對找出的HSV mask 進行輪廓搜尋: findContours
        # findContours 第一個輸入參數為 二值化的mask, 此處可以直接使用我們先前進行HSV過濾的結果
        # findContours 第二個輸入參數為 Flag, 可以設定要找最外圍的輪廓 或是連內部輪廓也一起尋找, 此處使用 RETR_EXTERNAL: 找最外圍的輪廓
        # findContours 第三個輸入參數為 亦為Flag, 可以設定要找全部的點或是僅找角點, 此處使用 CHAIN_APPROX_NONE: 尋找全部的點
        # findContours 會因為cv的版本而有不同的回傳, 如此處有發生問題,　請改成　　_, c_c, contour_h = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        c_c, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        # 由於 找出的 contours 可能很多個, 這裡假設在畫面中看到最大面積的輪廓即為 框
        # 透過 max function, 從c_c中根據contour面積作為key值: cv2.contourArea,找出 最大的面積的輪廓 max_c
        # max function 在遇上 空陣列 []會報錯, 導致程式終止, 如要避免此情況, 可以在執行max()前, 透過
        # if len(c_c) == 0:
        #   continue
        # 來 避免         
        max_c = max(c_c, key = cv2.contourArea)
        
        # 透過 minAreaRect找出一個可以包覆最大面積輪廓max_c 的最小方形
        # rect 會回傳
        # rect[0]: 中心點的座標, rect[0][0]為 中心點的座標x, rect[0][1]為 中心點的座標y
        # rect[1]: 為方形的 寬 高
        rect = cv2.minAreaRect(max_c)
        r_w, r_h = rect[1]
        rec_x = rect[0][0]
        rec_y = rect[0][1]

        # 建立變數 show_image, 用於顯示找到的輪廓與其中心點
        # 首先透過 np.zeros建立空的array, shape為讀取影像的大小, 種類為np.uint8
        # 再來透過 cv2.cvtColor, 將空array 從 gray_scale 格式轉換成 BGR 格式
        show_image = cv2.cvtColor(np.zeros(image.shape[:2], dtype=np.uint8), cv2.COLOR_GRAY2BGR)
        
        # 透過 rectangle 將一方形畫在 show_image上,  
        # (int(rec_x-0.5*r_w), int(rec_y-0.5*r_h)) 為方形左上角座標, 需為 int
        # (int(rec_x+0.5*r_w), int(rec_y+0.5*r_h)) 為方形右下角座標, 需為 int
        # (0,0,255)為方形線條顏色
        # -1的位置為方形線條寬度, -1 為填滿, 其餘則為 > 0 為線條寬度
        cv2.rectangle(show_image, (int(rec_x-0.5*r_w), int(rec_y-0.5*r_h)), (int(rec_x+0.5*r_w), int(rec_y+0.5*r_h)), (0,0,255), -1)
        
        # 顯示 方形中心點 於是窗上
        print("minAreaRect x: ", rec_x)
        print("minAreaRect y: ", rec_y)
        
        # 透過circle 將一圓形畫在show_image上,
        # (int(rec_x), int(rec_y)) 為圓心, 需為 int
        # 10的位置為 半徑
        # (0,50,175)的位置為顏色
        # -1的位置為圓形線條寬度, -1 為填滿, 其餘則為 > 0 為線條寬度
        cv2.circle(show_image, (int(rec_x), int(rec_y)), 10, (0,50,175), -1)
        
        # 透過 putText 將文字顯示在 show_image上
        # str((r_w * r_h) / (960*720.)) 為 方形寬 * 方形高 = 方形面積, (960*720.) 為 總畫面面積, 兩者相除後得到 方形面積在總畫面中的占比
        # 透過str()將float轉成 string
        # (10,40) 為文字顯示位置左上角, 5 為 字形代碼, 2 為文字大小, (255,255,0) 為顏色
        cv2.putText(show_image, str((r_w * r_h) / (960*720.)), (10,40), 5, 2, (255,255,0))
        
        # 當 start_detect 為 True
        if start_detect:
          # 當 mask面積 / 畫面總面積 (960 * 720) 大於等於 門檻值 0.35
          # 表示無人機已經夠接近框, 可以進行加速過框行為
          if (r_w * r_h) / (960*720.0) >= 0.35:
            # 透過 point_pub 發布 Float64MultiArray訊息, 將 框的 中心點x, 中心點y, 1發佈出去, 1表示占比已超過門檻值
            point_pub.publish(Float64MultiArray(data = [rec_x, rec_y, 1]))
            # 可加速通過框後就不再發佈訊息, 將 start_detect 改為 False
            start_detect = False
          # 當 mask面積 / 畫面總面積 (960 * 720) 小於 門檻值 0.35
          # 表示無人機還不夠接近框, 可以繼續進行校正與向前
          else:
            # 透過 point_pub 發布 Float64MultiArray訊息, 將 框的 中心點x, 中心點y, 0發佈出去, 0表示占比未超過門檻值
            point_pub.publish(Float64MultiArray(data = [rec_x, rec_y, 0]))            
        
        # 先透過 np.concatenate 將原始影像 image 以及視覺處理的影像 show_image 進行左右合併
        # 將合併後的影像 寫入到video: out中
        out.write(np.concatenate((image, show_image), axis = 1))    
        
        # 顯示 影像, 視窗名稱為result, 欲顯示的cv_img為 np.concatenate((image, show_image), axis = 1)
        cv2.imshow('result', np.concatenate((image, show_image), axis = 1))
        
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
