# JacobianKinematicsLab 硬點運動學

將靜態 XYZ 硬點轉成雙 A 臂懸吊，聯立求解四輪輪跳、前齒條轉向、轉向拉桿，以及剛性推桿與搖臂的運動。工具可整個資料夾單獨複製並啟動，使用 Python 3.10+、Tkinter 和 NumPy。

## 啟動與匯入

1. 在 Windows 雙擊 [Start-JacobianKinematicsLab.bat](Start-JacobianKinematicsLab.bat)。也可在工具資料夾的上一層執行以下指令。
2. 在新工具按「懸吊範例」，可載入由 [new/suspension_geometry.json](../new/suspension_geometry.json) 建立的 StaticWireframe 基準車輛。
3. 若要匯入修改後的幾何，在 Suspension Geometry Lab 開啟該 JSON，再按「運動學硬點 CSV」。匯出會合併四輪 A 臂、輪心，以及 JSON 裡的 `kinematics_hardpoints`。可從專案根目錄執行 `python -m suspension_geometry_lab` 開啟該工具。
4. 開啟「輪跳與轉向」，分別設定四輪輪心 Z 行程及前齒條沿軸位移，再按「開始求解」。可先用範例試算前齒條 +8 mm，並調整各輪振幅或行程形式。
5. 用 3D 視圖檢查硬點和桿件的運動，再到結果頁選擇節點、桿件或比較點。曲線可切換訊號，點擊曲線可定位樣本。CSV 可一次匯出全部節點和桿件的歷程，以及目前選取的兩點運動比。

```powershell
python -m pip install -r JacobianKinematicsLab/requirements.txt
python -m JacobianKinematicsLab
```

硬點表使用 UTF-8 CSV。表頭為 `name,x_m,y_m,z_m` 或 `name,x_mm,y_mm,z_mm`。也接受以 `point` 表示點名，並可加 `corner` 欄組合輪位與點名。每個點名須唯一。自訂機構 JSON 的點名須與 CSV 完全相同。

## 3D 運動視圖

在「3D 運動」頁可先檢查匯入的靜態硬點與桿件連接。求解完成後可使用「播放」、「暫停」與「回到起點」，也能調整播放速度、開啟「循環」，或拖曳時間軸停在指定樣本。3D 視圖依該樣本的 XYZ 座標畫出各節點，並依機構的桿件定義連接端點。

- 左鍵拖曳可旋轉，右鍵或中鍵拖曳可平移。滑鼠滾輪以游標處為中心縮放。
- 使用「3D 斜視」、「前視」、「側視」或「上視」核對不同方向的幾何。「重設視角」可回到目前視角的初始縮放與位置。
- 灰色方框是固定點，青綠色圓點是活動點，淡藍色線段是目前桿件。可勾選「靜態參考」疊加靜態姿態虛線，或勾選「節點名稱」顯示點名。
- 齒條、轉向拉桿與推桿以不同顏色呈現，可用視圖左上方的圖例辨識。
- 點選節點或桿件可用琥珀色標出，並顯示當下的節點 XYZ 或桿長。按「查看曲線」可開啟該節點或桿件的結果。

曲線、結果表與 3D 視圖共用同一個時間樣本。點擊曲線或選取表格列會暫停播放並定位到對應樣本，切回 3D 即可檢視該時間的幾何。

視圖同時顯示羊角的幾何參考線與輪向 `WHEEL_HEADING`，可用節點名稱或桿件名稱辨識。

## 懸吊硬點與工況

車輛座標為 X 向前、Y 向左、Z 向上。硬點名稱前面加輪位 `FL_`、`FR_`、`RL_` 或 `RR_`，例如 `FL_RACK`。可對照[完整四輪範例 CSV](examples/hardpoints.csv)。

基準車輛 JSON 目前採參數模式。範例的 24 個 A 臂硬點與 4 個輪心使用依現有懸吊參數和 `arm_mount_x` 算出的靜態座標。齒條、轉向拉桿、推桿與搖臂的 24 個示意點直接保存在同一份 JSON 的 `kinematics_hardpoints.corners`，單位為 m。分析實車時可在該區替換成量測座標，再重新匯出。

| 硬點後綴 | 定義 |
| --- | --- |
| `UCA_F`、`UCA_R`、`LCA_F`、`LCA_R` | 上、下 A 臂的前後車身固定點 |
| `UBJ`、`LBJ`、`WC` | 上下球接點與輪心，隨羊角運動 |
| `RACK`、`TOE` | 轉向拉桿內端與羊角外端。同一輪位須成對提供。前輪內端隨齒條移動，後輪內端固定在車身 |
| `PUSH_OUT` | 推桿外端，隨下 A 臂運動。接上 A 臂時改用 `PUSH_OUT_UCA`，接羊角時改用 `PUSH_OUT_UPRIGHT` |
| `PUSH_IN`、`ROCKER_A`、`ROCKER_B` | 推桿內端為搖臂活動點。兩個 `ROCKER` 點是車身固定鉸鏈的軸線定義，兩者須分開，且 `PUSH_IN` 須離開軸線 |
| `FORWARD` | 選填，以輪心指向此點作為靜態輪向。省略時採輪心前方 100 mm 的虛擬點 |

- 四輪輪心可各自設定 Z 行程。正值為上壓，負值為下垂。前齒條的輸入是沿軸的直線位移，單位為 mm。左右內端 `FR_RACK` 到 `FL_RACK` 定義齒條軸線，正值朝車輛左側。單輪前懸吊資料採 +Y 軸。轉向盤角度與齒條位移的換算需使用實車的轉向機構資料。
- 各輪與前齒條可分別選「往返」、「正弦」或「線性」。往返從零到輸入振幅再回零，正弦走完一個正負週期，線性等速從零移至輸入量。基準範例預設輪心上壓 +20 mm、21 個樣本及齒條 0 mm。「基準上壓 +20 mm」和「基準下垂 −30 mm」可把四輪行程切換至來源 JSON 的上壓或下垂設定。來源的轉向掃描角度用於 Suspension Geometry Lab，齒條的直線行程可在新工具另行輸入。
- 在「桿件」可選 `FL_WHEEL_HEADING`、`FR_WHEEL_HEADING` 等輪向參考線，方位角由車輛 +X 朝 +Y 為正。亦可查看 `FL_RACK-TOE`、`FL_PUSH_ROD` 等桿件的角度、角速度與角加速度。推桿兩端保持固定桿長，內端隨搖臂鉸鏈轉動。
- 基本懸吊 CSV 仍可匯入。若當輪只有 A 臂硬點與輪心，會維持靜態輪向。只含六個 A 臂硬點時，行程輸入控制上球接點的 Z。舊表的 `UFI`、`URI`、`LFI`、`LRI`、`wheel_center`、`ROD_IN`、`ROD_OUT` 可直接匯入。

## 自訂懸吊機構

非標準接法可同時載入硬點 CSV 與機構 JSON，為各點指定剛體附著、固定距離、投影和驅動。

| JSON 欄位 | 用法 |
| --- | --- |
| `fixed` | 車身或地面固定點的名稱陣列 |
| `links` | 每條剛性距離約束包含 `name`、端點 `a`、端點 `b`，長度取自靜態硬點。可用 `constrained: false` 保留僅供顯示與輸出的參考線 |
| `attachments` | 選填。每項包含活動點 `point` 與三個非共線錨點 `a`、`b`、`c`。由其他約束維持錨點三角形的尺寸，附著點隨該剛體移動 |
| `drives` | 每項包含 `name`、`point`、`direction`，以指定方向上的投影相對靜態位置作為驅動座標 |
| 驅動的 `b` | 選填，用兩點 `point − b` 的相對位置投影，適用於保持某一方向的相對間距 |
| `amplitude_mm`、`profile` | 振幅以 mm 表示。 `pulse` 為平滑往返，`sine` 為完整正弦週期，`ramp` 為等速單程 |
| `locked` | 設為 `true` 時固定投影值，振幅保持零，可用於齒條導軌或限制相對方向 |
| `duration_s`、`samples` | 工況時間與樣本數，預設 2 秒、121 點 |

多點剛體可用足以確定其形狀的桿長約束組成。模型應給足約束與驅動，使每個移動座標在局部唯一。若起始姿態秩不足、桿長不相容、行程不可達或作動經過奇異姿態，求解會顯示原因及對應時間（適用時）。

## 結果與計算

| 項目 | CSV | 單位與定義 |
| --- | --- | --- |
| 節點 | `point_kinematics.csv` | 每個時間、每個點的 XYZ 位置、速度與加速度，採 m、m/s、m/s²。介面採 mm、mm/s、mm/s² |
| 桿件 | `rod_kinematics.csv` | 每條桿件從 A 指向 B 的方位角、仰角、與靜態方向的夾角（°），以及桿軸三軸角速度和角加速度（rad/s、rad/s²） |
| 兩點比較 | `motion_ratio.csv` | 所選點與投影方向、投影位移、切線率、有號運動比、理想機械利益與各樣本狀態 |

位置以靜態姿態為起點，逐步沿驅動歷程迭代解固定桿長與投影約束。速度使用同一個約束 Jacobian 解一階微分方程。加速度使用二階微分，並納入各桿件端點相對速度造成的曲率項。桿軸由兩端點決定，角速度與加速度描述軸線轉向。求桿件繞自身長軸的自旋需要額外姿態資訊。方位角在各段連續展開，桿軸鉛直的樣本留白。

兩點比較可各自指定量測方向，例如 `z`、`-y` 或 `1,0,2`。先把兩個點的虛位移分別投影成 `ds_in` 與 `ds_out`。運動比為 `ds_out / ds_in`，保留方向正負。理想機械利益依虛功關係為 `|ds_in / ds_out|`。兩者均無因次。

有多項驅動時，可沿實際時間歷程比較，也可選定一項驅動，以 Jacobian 求該驅動的局部偏導數並保持其他驅動座標固定。沿時間歷程比較時，往返端點的速度可能同時為零，此處留白。指定單項驅動可在同一姿態檢查虛位移比。分母投影為零的樣本留白，CSV `status` 欄記錄原因。

## 批次求解

```powershell
python -m JacobianKinematicsLab --hardpoints JacobianKinematicsLab/examples/hardpoints.csv --output kinematics-results --amplitude FL_WC_Z 20 --amplitude FR_WC_Z -10 --amplitude FRONT_RACK 8 --profile FRONT_RACK sine --pair FL_WC FL_PUSH_IN --ratio-drive FL_WC_Z
```
