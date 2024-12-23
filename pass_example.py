#!/usr/bin/env python
# -*- coding: utf-8 -*-

### 範例程式 ###
### 使用test_h264_sub.py 找出框的中心點, 並在pass_example.py中進行移動控制, 使無人機的影像中心點 (480, 200) 與找到框中心點能盡量重合 ###
### 重合後即可向前移動 ###
### 當canPass 為 1 時, 表示影像程式認為 已經夠接近框,在靠近會沒辦法看到完整的框, 這時進行加速通過的行為, 並終止控制迴圈 ###
### 使用simple_tello.py 來建立Tello與控制Tello執行相應動作 ###

import rospy                         # 導入套件: rospy
import simple_tello                  # 導入 simple_tello
from geometry_msgs.msg import Twist  # 導入 geometry_msgs 裡的 Twist 訊息格式
from std_msgs.msg import Empty       # 導入 std_msgs 裡的 Empty 訊息格式

# 定義function tello_pass: 用於進行控制迴圈
# 控制迴圈為一while 迴圈, 在迴圈中會根據 target_x  target_y  canPass 進行相應動作
# target_x: 藉由訂閱topic 所獲得的框中心點 x 
# target_y: 藉由訂閱topic 所獲得的框中心點 y
# canPass: 藉由訂閱topic 所獲得是否mask面積超過門檻值
# 在此處我們定義 較小的門檻值 24, 表示 target_x 與 中心點x 480相減要 小於  24,才認定有重合
# 在此處我們定義 較小的門檻值 24, 表示 target_y 與 中心點y 200相減要 小於  24,才認定有重合
# 當認定重合後, 我們以一個較寬鬆的門檻值 60 來檢查 target_x 與 中心點x 480 是否脫離重合, 如有脫離則需要重新校正左右
# 當認定重合後, 我們以一個較寬鬆的門檻值 30 來檢查 target_y 與 中心點y 200 是否脫離重合, 如有脫離則需要重新校正上下
def tello_pass(t1):
  
  # 建立重合確認變數 check, 用於目前是否重合
  check = False
  
  # 開始執行控制迴圈, 終止條件為 roscore停止時, 或是被break
  while not rospy.is_shutdown():
    
    # 檢查 target_x 與 target_y 是否有收到訂閱值, 沒有收到的情況下皆為 -1, 透過 while 迴圈 讓程式卡在此處
    while t1.state.target_x == -1 and t1.state.target_y == -1:
      pass
    
    # 當 target_x 與 target_y 不為 -1, 表示有收到訂閱值, 透過print 顯示收到的訂閱值
    print(t1.state.target_x, t1.state.target_y)
    
    # 計算 target_x 與 中心點x 480 之間的差
    dx = t1.state.target_x - 480
    
    # 計算 target_y 與 中心點y 200 之間的差
    dy = t1.state.target_y - 200
    
    # 當canPass 為 1, 表示影像程式認為mask 面積已達到門檻值, 故此時執行加速過框的行為
    if t1.state.canPass == 1:
      
      # 建立 Twist 訊息, 並給予向前速度 0.4, 持續3秒
      msg = Twist()
      msg.linear.y = 0.4
      #msg.linear.z = 0.1
      t1.controler.move(msg, 3)
      
      # 建立 Twist 訊息, 並給予向前速度 0.5, 持續3秒 
      msg = Twist()
      msg.linear.y = 0.5
      t1.controler.move(msg, 3)
  
      # 建立 Twist 訊息, 未設定值表示懸停(不移動不旋轉), 持續1秒
      msg = Twist()
      t1.controler.move(msg, 1)
      break # 中斷控制迴圈
    
    # 當canPass 為 0, 表示影像程式認為mask 面積未達到門檻值, 此時我們根據 target_x 480 target_y 200進行左右 上下校正, 當重合時進行向前
    elif t1.state.canPass == 0:
      
      # 當尚未重合時, 以較小的區間來判斷是否重合
      if check == False:
        # 當 絕對值 (target_x 與 480 的差) 小於 24, 表示 左右重合
        # 當 絕對值 (target_y 與 200 的差) 小於 24, 表示 上下重合
        # 當左右 上下都重合, 表示已重合, 將check設置為True, 並開始向前        
        if abs(dx) < 24 and abs(dy) < 24:
          check = True
          
          # 建立 Twist 訊息, 並給予向前速度 0.2, 持續0.5秒 
          msg = Twist()
          msg.linear.y = 0.2
          t1.controler.move(msg, 0.5)
        # 左右 上下沒有都重合時, 根據 target_x 與 480的差 以及 target_y 與 200的差進行左右 上下的定速校正
        else:
          # 建立 Twist 訊息
          msg = Twist()
          
          # 當dx (target_x 與 480的差) 不等於0時, 根據其方向進行左右校正, linear.x: 左- 右+, dx / abs(dx) 表示取其正負號, 亦可用np.sign來取得   
          # 480 為影像中心點 x, 大於 480 表示在中心點的右邊, 小於 480表示在中心點的左邊, 因此當target_x - 480後, +表示要往右邊走, -表示要往左邊走
          if dx != 0:
            msg.linear.x = dx / abs(dx) * 0.1
          
          # 當dy (target_y 與 200的差) 不等於0時, 根據其方向進行上下校正, linear.z: 下- 上+, dy / abs(dy) 表示取其正負號, 亦可用np.sign來取得   
          # 200 為影像中心點 y, 大於 200 表示在中心點的下方, 小於 200表示在中心點的上方, 因此當target_y - 200後, +表示要往下方走, -表示要往上方走, 但這與linear.z的方向相反,因此需要加一個負號來使其一致
          if dy != 0:
            msg.linear.z = -dy / abs(dy) * 0.2
          
          # 依照設定好的方向與速度持續發送0.5秒
          t1.controler.move(msg, 0.5)
      
      #  重合時, 以較大的區間來判斷是否脫離重合
      else:
      
        #當 dx (target_x 與 480的差) 超過 60 表示 脫離 重合, 需要重新進行重合
        #當 dy (target_y 與 200的差) 超過 30 表示 脫離 重合, 需要重新進行重合
        #將check設置為False, 下次可以使用較小的區間去判斷重合
        if abs(dx) >= 60 or abs(dy) >= 30:
          check = False
          
          # 建立 Twist 訊息
          msg = Twist()
          
          # 當dx (target_x 與 480的差) 不等於0時, 根據其方向進行左右校正, linear.x: 左- 右+, dx / abs(dx) 表示取其正負號, 亦可用np.sign來取得   
          # 480 為影像中心點 x, 大於 480 表示在中心點的右邊, 小於 480表示在中心點的左邊, 因此當target_x - 480後, +表示要往右邊走, -表示要往左邊走
          if dx != 0:
            msg.linear.x = dx / abs(dx) * 0.1
          # 當dy (target_y 與 200的差) 不等於0時, 根據其方向進行上下校正, linear.z: 下- 上+, dy / abs(dy) 表示取其正負號, 亦可用np.sign來取得   
          # 200 為影像中心點 y, 大於 200 表示在中心點的下方, 小於 200表示在中心點的上方, 因此當target_y - 200後, +表示要往下方走, -表示要往上方走, 但這與linear.z的方向相反,因此需要加一個負號來使其一致
          if dy != 0:
            msg.linear.z = -dy / abs(dy) * 0.2
          
          # 依照設定好的方向與速度持續發送0.5秒
          t1.controler.move(msg, 0.5)
        
        #當 dx (target_x 與 480的差) 小於 60 表示 未脫離 重合 且 當 dy (target_y 與 200的差) 小於 30 表示 未脫離 重合        
        else:
        
          # 建立 Twist 訊息, 並給予向前速度 0.2, 持續0.5秒 
          msg = Twist()
          msg.linear.y = 0.2
          t1.controler.move(msg, 0.5)
        
      ### 因控制迴圈, 執行到此處後會回到上面重新進行 dx 與 dy的計算, 並根據 target_x  target_y  canPass 進行相應動作 ###
 
# main function
# 負責執行一連串的指令, 如 起飛 開始過框 降落
def main():
  
  t1 = simple_tello.Tello_drone() #建立 simple_tello.Tello_drone 變數 t1
  
  while t1.state.is_flying == False:  # 當is_flying為False, 表示無人機還沒有成功起飛, 故透過while 迴圈不斷執行起飛, 直到is_flying為True, 表示無人機起飛成功
    t1.controler.takeoff()
  
  while t1.state.fly_mode != 6: # 當fly_mode != 6, 表示無人機正在執行特殊行為, 如 起飛 降落 翻滾等等, 因此透過while迴圈確認特殊行為是否結束
    print("wait...")
  
  tello_pass(t1) # 執行過框的控制迴圈 function: tello_pass, 會使用到 t1 作為輸入變數
  
  while t1.state.fly_mode != 6: # 當fly_mode != 6, 表示無人機正在執行特殊行為, 如 起飛 降落 翻滾等等, 因此透過while迴圈確認特殊行為是否結束
    print("wait...") 
    
  while t1.state.is_flying == True:  # 當is_flying為True, 表示無人機還正在飛行, 故透過while 迴圈不斷執行降落, 直到is_flying為False, 表示無人機完成降落
    t1.controler.land() 

# main
if __name__ == "__main__":

  # 告訴ros此程式為node, node名稱為 'pass_example', anonymous為True,表示ros會幫此node加一段亂碼進行匿名
  rospy.init_node("pass_example", anonymous=True)
  
  #執行 範例function main
  main()
