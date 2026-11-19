# 反向動力學求解器
## [DEMO](introduction/DEMO_settings.md)

    範例模擬程序，說明設定作業與各種功能設定。雖然有UI版本設定json的模擬工具，但是還是保留平坦未模組化的使用範例。

## [多體引擎](introduction/Multibody_engine.md)

    多體系統引擎 (Multibody_engine.py)：負責定義系統中的節點（Node）和桿件（Rod），建立幾何約束關係，並處理視覺化渲染。支持固定點與自由點的區分，便於建模複雜的機械結構。

## [靜力約束引擎](introduction/Static_engine.md)

    靜平衡態引擎 (Static_engine.py)：專注於靜態平衡分析，計算系統在靜止狀態下的力平衡與約束條件。也負責求解靜態系統的受力情況

## [材料力學引擎](introduction/Material_engine.md)

    材料力學引擎 (Material_engine.md):透過靜態受力進一步分析部件內應力與受力形變。進一步判斷部件材料狀況。

## [反向運動學引擎](introduction/Kinematics_engine.md)

    運動學引擎 (Kinematics_engine.py)：核心求解器，處理速度與加速度的數值計算。利用雅可比矩陣（Jacobian）進行線性最小二乘求解，支持數值微分估計運動狀態。

## [虛功引擎](introduction/Virtual_work_engine.md)

    虛功引擎 (Virtual_work_engine.py)：基於虛功原理進行動力學分析，適用於能量平衡與優化計算。

## [核心模擬器](introduction/simulator.md)

    模擬器 (simulator.py)：整合所有引擎，提供時間步進模擬功能，支持動態輸入軌跡與實時視覺化輸出。

## [幾何學模組](introduction/Geometry.md)
    幾何 (Geometry.py)：進行常見的解析幾何學運算 。

## [運動模組](introduction/motion.md)
    運動 (motion.py)：用於產生各種運動輸入軌跡，將讓時間成為運動軌跡函數。

## [截面模組](introduction/shape.md)
    形狀 (shape.py) : 進行截面相關計算，主要用於材料力學面的相關計算，沒有直接於其他模組連通，但計算結果可以輸入data表格提升效率

## [數據與資料](introduction/output.md)
    輸出資料可記錄成csv、圖表、文字報告，另外也有工具進行後續的分析處理。