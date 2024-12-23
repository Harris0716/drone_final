#!/usr/bin/env python
# -*- coding: utf-8 -*-

### 此為行為樹behavior tree範例程式 ### 
### 該程式針對先前的pass_example.py 使用行為樹進行修改, ### 
### 該程式針對無人機過框的過程區分為3個條件節點, 6個動作節點 ###
### 定義了節點的階層與順序 ###
### >> 表示順序節點 Sequence node
### | 表示選擇節點 Selector node
### 請搭配先前的 pass_example中的 test_h264_sub.py 以及 simple_tello.py 使用 ###
### behave 請放在同一目錄下 ###
### 如有新增節點, 建議可以在節點加入 print, 觀察其在 terminal 上 的流程 ###
 
import rospy                        # 導入套件: rospy
import simple_tello                 # 導入 simple_tello

from geometry_msgs.msg import Twist # 導入 geometry_msgs 裡的 Twist 訊息格式
from std_msgs.msg import Empty      # 導入 std_msgs 裡的 Empty 訊息格式
from behave import *                # 導入 behave 中的所有class
from time import sleep              # 導入 time 裡的 sleep

#建立 simple_tello.Tello_drone 變數 t1
t1 = simple_tello.Tello_drone()

# class bt_mission
# 建立 基於行為樹之任務模型
class bt_mission:

    # bool 變數 isContinue, 當 True 時 bt 會持續 tick, False 時則會停止tick, 並回到main中往後執行
    isContinue = True
    
    # 定義 影像的中心點座標, 影像大小為 (960, 720)
    center = (480, 200)

    # 初始 dx, dy 
    dx = -1
    dy = -1

    # 初始化 
    # 定義過框的行為樹 self.tree
    # 樹的樣貌請見 課程投影片 behavior tree1, Behavior tree for pass 
    def __init__(self):
    
        # behavior tree
        # 透過 | 進行 三個子樹之間的切換
        # 第一個子樹
        #   (self.isNotDataReceived >> self.doHover)  先用條件節點確認是不是沒有收到資料,是的話就做動作節點 doHover 進行懸停
        # 第二個子樹
        #   (self.isReceivePass >> self.doAddSp >> self.doBtStop)  如果第一個子樹的條件節點錯誤, 就會被導引到第二個子樹, 
        #   確認 canPass 是否為 1, 是的話則 進行 動作節點doAddSp 加速過框, 並將  isContinue 設為False, 終止bt的tick
        # 第三個子樹 
        #   (self.doComputeData >> (self.isForward >> self.doForward) | (self.doCorrection)) 
        #   當第二個子樹的條件節點 isReceivePass 發現 canPass 為 0, 則會導引到第三個子樹
        #   先到動作節點 doComputeData 進行 dx 與 dy 的計算, 計算完後 根據 >> sequence node 導引 到 子樹 (self.isForward >> self.doForward) | (self.doCorrection)
        #   子樹會先確認條件節點 self.isForward 是否滿足, 滿足的話則進行動作節點 self.doForward
        #   如self.isForward不滿足條件, 則子樹裡的 | 將導引到最後一個動作節點 self.doCorrection 進行校正, 使畫面中心點 與 找出的框中心點 可以盡量重合
        self.tree = (
        
            (self.isNotDataReceived >> self.doHover)
            | (self.isReceivePass >> self.doAddSp >> self.doBtStop)
            | (self.doComputeData >> (self.isForward >> self.doForward) | (self.doCorrection)) 
        )    

    # 透過 @condition 定義此 function 為條件節點
    # 在條件節點內可以直接回傳 True 或是 False
    # 如有需要做顛倒可以使用 not()
    # isNotDataReceived 判斷 是否沒有收到 test_h264_sub所發佈的框中心點座標資料
    # 因此在 return 時 就判斷 target_x 與 target_y 是否 皆為預設值
    @condition
    def isNotDataReceived(self):
        print("condition: isNotDataReceived")
        return t1.state.target_x == -1 and t1.state.target_y == -1

    # 透過 @action 定義此 function 為動作節點
    # 一般動作節點不需要寫 return , bt會默認 其做完就是 True了
    # 如果有需要, 可以透過 return  FAILURE 來使 動作節點 回傳 失敗
    # 如果有需要, 可以透過 return  SUCCESS 來使動作節點提前結束
    # doHover 為一動作節點, 顧名思義執行懸停行為
    @action
    def doHover(self):
        print("action: doHover")

        # 建立 Twist 訊息, 以 預設值 0 發送 0.5秒 
        msg = Twist()
        t1.controler.move(msg, 0.5)

    # 透過 @condition 定義此 function 為條件節點
    # 在條件節點內可以直接回傳 True 或是 False
    # 如有需要做顛倒可以使用 not()
    # isReceivePass 判斷 canPass 是否為 1
    # 透過 return 條件式判斷:  t1.state.canPass == 1
    @condition
    def isReceivePass(self):
        print("condition: isReceivePass")
        return t1.state.canPass == 1

    # 透過 @action 定義此 function 為動作節點
    # 一般動作節點不需要寫 return , bt會默認 其做完就是 True了
    # 如果有需要, 可以透過 return  FAILURE 來使 動作節點 回傳 失敗
    # 如果有需要, 可以透過 return  SUCCESS 來使動作節點提前結束
    # doAddSp 為一動作節點, 顧名思義執行加速行為
    @action
    def doAddSp(self):
        print("action: doAddSp")
        
        # 建立 Twist 訊息, 並給予向前速度 0.4, 持續3秒
        msg = Twist()
        msg.linear.y = 0.4
        t1.controler.move(msg, 3)
      
        # 建立 Twist 訊息, 並給予向前速度 0.5, 持續3秒 
        msg = Twist()
        msg.linear.y = 0.5
        t1.controler.move(msg, 3)

    # 透過 @action 定義此 function 為動作節點
    # 一般動作節點不需要寫 return , bt會默認 其做完就是 True了
    # 如果有需要, 可以透過 return  FAILURE 來使 動作節點 回傳 失敗
    # 如果有需要, 可以透過 return  SUCCESS 來使動作節點提前結束
    # doBtStop 為一動作節點, 顧名思義將 isContinue 設置為 False, 使 bt 不在繼續 tick
    @action
    def doBtStop(self):
        print("action: doBtStop")
        bt_mission.isContinue = False

    # 透過 @action 定義此 function 為動作節點
    # 一般動作節點不需要寫 return , bt會默認 其做完就是 True了
    # 如果有需要, 可以透過 return  FAILURE 來使 動作節點 回傳 失敗
    # 如果有需要, 可以透過 return  SUCCESS 來使動作節點提前結束
    # doComputeData 為一動作節點, 顧名思義計算 dx 與 dy
    # dx: 計算 target_x 與 中心點x 480 之間的差
    # dy: 計算 target_y 與 中心點y 200 之間的差
    @action
    def doComputeData(self):
        print("action: doComputeData")
        bt_mission.dx = t1.state.target_x - bt_mission.center[0]
        bt_mission.dy = t1.state.target_y - bt_mission.center[1]
        print("doComputeData: ", bt_mission.dx, bt_mission.dy)
     
    # 透過 @condition 定義此 function 為條件節點
    # 在條件節點內可以直接回傳 True 或是 False
    # 如有需要做顛倒可以使用 not()   
    # isForward 為一條件節點, 會判斷 dx 與 dy  是否 再設定的門檻值內, 
    # dx 與 dy 皆小於門檻值, 表示 畫面中心點與框中心點重合
    # 若 dx 與 dy其中一個沒有小於門檻值, 表示 畫面中心點與框中心點沒有重合
    @condition
    def isForward(self):
        print("condition: isForward")
        return abs(bt_mission.dx) < 30 and abs(bt_mission.dy) < 30
    
    # 透過 @action 定義此 function 為動作節點
    # 一般動作節點不需要寫 return , bt會默認 其做完就是 True了
    # 如果有需要, 可以透過 return  FAILURE 來使 動作節點 回傳 失敗
    # 如果有需要, 可以透過 return  SUCCESS 來使動作節點提前結束
    # doForward 為一動作節點, 顧名思義進行 向前行為
    @action
    def doForward(self):
        print("action: doForward")
        
        # 建立 Twist 訊息, 並給予向前速度 0.2, 持續0.5秒
        msg = Twist()
        msg.linear.y = 0.2
        t1.controler.move(msg, 0.5)
    
    # 透過 @action 定義此 function 為動作節點
    # 一般動作節點不需要寫 return , bt會默認 其做完就是 True了
    # 如果有需要, 可以透過 return  FAILURE 來使 動作節點 回傳 失敗
    # 如果有需要, 可以透過 return  SUCCESS 來使動作節點提前結束
    # doCorrection 為一動作節點, 顧名思義進行 上下 左右 校正行為
    
    @action
    def doCorrection(self):
        print("action: doCorrection")
        
        # 建立 Twist 訊息
        msg = Twist()
        # 當dx (target_x 與 480的差) 不等於0時, 根據其方向進行左右校正, linear.x: 左- 右+, dx / abs(dx) 表示取其正負號, 亦可用np.sign來取得   
        # 480 為影像中心點 x, 大於 480 表示在中心點的右邊, 小於 480表示在中心點的左邊, 因此當target_x - 480後, +表示要往右邊走, -表示要往左邊走
        if bt_mission.dx != 0:
          msg.linear.x = bt_mission.dx / abs(bt_mission.dx) * 0.1
        
        # 當dy (target_y 與 200的差) 不等於0時, 根據其方向進行上下校正, linear.z: 下- 上+, dy / abs(dy) 表示取其正負號, 亦可用np.sign來取得   
        # 200 為影像中心點 y, 大於 200 表示在中心點的下方, 小於 200表示在中心點的上方, 因此當target_y - 200後, +表示要往下方走, -表示要往上方走, 但這與linear.z的方向相反,因此需要加一個負號來使其一致    
        if bt_mission.dy != 0:
          msg.linear.z = -bt_mission.dy / abs(bt_mission.dy) * 0.2
          
        # 依照設定好的方向與速度持續發送0.5秒
        t1.controler.move(msg, 0.5)
    
    # 定義function run: 用於執行 行為樹的tick
    def run(self):
        
        # while loop, 會持續運作, 直到程式關掉或是break中斷
        while True:
            
            # 當 isContinue 為 False, 透過 break 中斷while 迴圈
            if bt_mission.isContinue == False:
                break
            # 行為樹設置 與 tick
            bb = self.tree.blackboard(1)
            state = bb.tick()
            
            # 顯示目前的行為樹運行的節點狀態 SUCCESS or FAILURE
            print("state = %s\n" % state)
            
            # 當 state還在運行時: RUNNING
            while state == RUNNING:
            
                # 持續 tick 
                state = bb.tick()
                
                # 顯示目前的行為樹運行的節點狀態 SUCCESS or FAILURE
                print("state = %s\n" % state)
            
            # 透過 assert (斷言) 來進行 條件判斷, 在做完全部的tick後, state 應該要是 SUCCESS 或是 FAILURE, 而不是 RUNNING    
            assert state == SUCCESS or state == FAILURE

# main function
# 負責執行一連串的指令, 如 起飛 開始過框之behavior tree  降落
def main():
    
    while t1.state.is_flying == False:  # 當is_flying為False, 表示無人機還沒有成功起飛, 故透過while 迴圈不斷執行起飛, 直到is_flying為True, 表示無人機起飛成功
      t1.controler.takeoff()
  
    while t1.state.fly_mode != 6: # 當fly_mode != 6, 表示無人機正在執行特殊行為, 如 起飛 降落 翻滾等等, 因此透過while迴圈確認特殊行為是否結束
      print("wait...")
 
    # 建立 bt_mission 物件 btCm_n
    btCm_n = bt_mission()
    
    # 建立後等待2秒, 讓 物件裡面的值, rospy.Subscriber 初始化完成
    sleep(2)
    
    # 顯示 "bt start..."
    print("bt start...")
    
    # 執行 btCm_n.run, 讓bt 開始 tick
    btCm_n.run()
    
    while t1.state.fly_mode != 6: # 當fly_mode != 6, 表示無人機正在執行特殊行為, 如 起飛 降落 翻滾等等, 因此透過while迴圈確認特殊行為是否結束
      print("wait...") 
    
    while t1.state.is_flying == True:  # 當is_flying為True, 表示無人機還正在飛行, 故透過while 迴圈不斷執行降落, 直到is_flying為False, 表示無人機完成降落
      t1.controler.land()        

# main
if __name__ == "__main__":

    # 告訴ros此程式為node, node名稱為 'h264_pub', anonymous為True,表示ros會幫此node加一段亂碼進行匿名
    rospy.init_node('h264_pub', anonymous=True)
    
    #執行 範例function main
    main()
