# TODO

## 1
目前我們的軌跡求解器可以求解兩個自由度，也就是兩個輸入，如同json中的"outer_name": "arm","inner_name": "rack",但是我可能提供1,2,3,...甚至更多個自由度的輸入。我需要這個架構可以支援一個或多個輸入，而非只能進行兩個輸入。請幫我進行修改
1. trajectory_solver_V3.py: sweep需要能夠支援一個輸入或多個輸入
2. output.py: 輸出的json和csv都需要可以支援一個或多個輸入
3. 請幫我建立兩個demo_simulation:一個add_driver(arm, rack)另一個add_driver(rack)並且兩個都需要能夠正常運作


## 2
不太正確，我需要支援單個輸入和多個輸入其中資料串必須這樣紀錄，同時csv與json資料數量需要正確，舉例來說input1 step 30 & input2 step 100 總資料長會是3000，而如果只有 舉例來說input1 step 100 總資料長就會是100
* 多輸入
      "input1_step": 0,
      "input2_step": 0,
            "nodes": {...
      "input1_step": 1,
      "input2_step": 0,
            "nodes": {...            
* 單輸入
      "input1_step": 0,
            "nodes": {...
      "input1_step": 1,
            "nodes": {...

## 3
接下來我希望可以透過讀取node_position.csv 的檔案重現機構狀態，node_position.csv 中的資料是我們之前已經預先計算好的結果。我需要有辦法定義其中一個點然後直接得出當下狀態的機構，其中不透過重新的約束求解。
以下是我希望建立的目標:
建立物件並讀取路徑
1. 透過套用loader進行物件載入，將node rod wheel 載入
2. 讀取path.csv其中t為時間後續為物件名稱與xyz，如下
rack_x,rack_y,rack_z,wheel_o_x,wheel_o_y,wheel_o_z
---
建立node_position讀取工具
1. 一個新的python工具可以讀取node_position.csv 然後取出指定的機構狀態如果node_position沒有剛好指定的數值就透過插植進行求解
2. 透過傳入指定的物件參數從node_position中找出其他物件的狀態並輸出
3. node_position中可以讀取driver的數量，需判斷driver與輸入物件數量
    if 輸入物件數量 少於 driver數量
    回傳輸入不足錯誤
4. 透過path中的欄位名稱查找node_position的欄位，找出最相近的情況當起始點並用driver的欄位尋找一種到下一步後的之狀態，用最匹配的狀態進行插植進行求解
指定輸入與查表

建立渲染
1. 將物指定位置傳入json讀取工具，並利用輸出結果給vpython_renderer進行渲染。

---
建立兩個python檔案

載入路徑path.csv
1. 讀取path
2. 讀取欄位標題其中，t是時間，其他欄位會是node物件的xyz座標，例如:wheel_o_x
3. 得知只知物件的xyz並判斷已知數量，例如:rack_x,rack_y,rack_z,wheel_o_x,wheel_o_y,wheel_o_z有兩個已知node條件

載入幾何關係node_position.csv
1. 讀取node_position
2. 找出 driver數量: 從標題欄位中找出含有driver的key名稱
3. 取得只知條件輸入，可能是任何一個點條件(也有可能是driver的數值)
4. 已第一個