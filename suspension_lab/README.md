# Suspension Lab 懸吊幾何工作台

本工具是獨立的五頁 Tkinter 桌面介面，沿用 Vehicle Lab 的 Canvas 圖表、平移縮放與匯出方法。完整靜態構造直接呼叫套件內的 `static/fv/wireframe_geometry.py`，與套件內的 `static/fv/tk_wireframe.py` 使用相同來源。FV、SV 的運動零位均來自這份靜態結果；WE3D 使用其中合併 FV / SV 的真實 3D 主銷。

幾何程式已收進 `suspension_lab/static/`，輪端旋轉函式位於 `suspension_lab/wheel_end_geometry.py`；全部使用套件內 import，不再將 `new` 加入 Python 搜尋路徑，也不依賴其中任何 Python 檔案。移除 `new/static`、`new/suspension_geo` 或整個 `new` 資料夾不影響本工具。仍須保留共用圖表與匯出模組所在的 `vehicle_lab`。

## 啟動與參數操作

在專案根目錄雙擊 `Start-SuspensionLab.bat`，或執行：

```powershell
python -m suspension_lab
python -m suspension_lab --project projects/my_suspension.json
```

獨立靜態線框預覽可雙擊 `Start-StaticWireframe.bat`，或執行 `python -m suspension_lab.static.fv.tk_wireframe`；同樣使用套件內幾何程式。

需要 Python 3.10+、Tkinter 與 NumPy；PNG 匯出需要 Pillow。這台電腦的 Codex Python 已具備套件。其他環境可安裝：

```powershell
python -m pip install -r requirements-suspension.txt
```

1. 按右上「共用車輛參數」，可在車輛 / 輪胎、前軸、後軸、掃描範圍四個頁籤編輯全部參數，再按「套用全部參數」。
2. 各分析頁左側也可編輯相關設計，按「套用左側參數並重新解算」後，五頁會同步使用新設定，舊結果與掃描快取會清除。未套用的輸入框內容尚不屬於專案。
3. 按「儲存專案」或 Ctrl+S 儲存 JSON；「另存專案」可保留不同設計版本。「開啟專案」或 Ctrl+O 導入。檔名旁的 `*` 表示已套用設定尚未儲存。
4. 選擇圖表，拖動滑桿觀察目前姿態。拖曳過程會持續求解、顯示已完成的姿態，再接續求解最新滑桿位置，不必放開滑鼠。曲面與範圍圖是整個設定範圍的掃描，不會隨目前姿態改變掃描邊界。
5. 3D 圖拖曳旋轉，2D 圖拖曳平移；滾輪縮放。幾何圖與 RC / PC 位置分布圖的 X、Y 軸維持相同單位的等比例，視窗縮放後也會維持。滑桿更新時保留目前縮放、平移與視角；按「重設視角」或雙擊才會重新置中。切換圖表、座標模式、車軸或套用新設計時會重新配合圖形範圍。結果表可捲動查看所有數值。

基準專案為 `default_project.json`，數值沿用 `static/car.py` 的原始設定，包括前後 FVSA 1 m、RC 35 / 40 mm、KPI 12 / 13 deg、caster 5 / 0 deg。工具的可編輯設計值只讀 JSON；套件內的 Car / Suspension 類別只用作靜態求解器的型別介面，不會呼叫它們的預設值建構子。獨立靜態線框預覽仍可使用原有設定。

本工具與原 Vehicle Lab 的 JSON 結構不同；兩者用途、硬點來源與參數種類不同，開啟錯誤格式會明確顯示錯誤。

## 五個頁面

| 頁面 | 滑桿 | 可切換圖表 |
|---|---|---|
| 前 FV | 左 / 右上壓行程，或 heave / roll | 動態機構、RC x′ / y′ 位置分布、RC 高度 y′ 曲面、RC 水平位置 x′ 曲面、左右 camber 與 camber gain 曲面 |
| 後 FV | 與前 FV 相同，使用獨立後軸設計 | 與前 FV 相同 |
| SV | 前 / 後上壓行程，或 heave / pitch | 動態機構、前後 caster、anti-dive、anti-lift、pitch center 高度 y′ / 位置 x′ 曲面、PC x′ / y′ 位置分布 |
| WE3D | steering，可選前 / 後軸 | 輪端 3D、steering × 設計 caster → 輪軸傾角曲面、接地半徑、接地 XYZ、3D 軌跡、caster / 傾角、三方向轉向力臂 |
| 靜態全車 | 旋轉與縮放 | 全車 3D / 前視 / 側視 / 上視，獨立開關前 FV、後 FV、SV 平面 |

靜態全車上方的「瞬心／Kingpin 連線與延長線」可一鍵隱藏或顯示瞬心參考線、瞬心與主銷接點組成的參考三角形，以及主銷延長線。切換會保留目前縮放與視角，PNG、SVG 與報告沿用目前顯示設定；硬點資料仍可完整匯出。

FV / SV 的「滑桿 / 曲面座標」選單同時決定掃描座標。FV 可用左右行程或 heave / roll；SV 可用前後 ride height 或 heave / pitch。SV 在前後行程模式下，滑桿控制輪側上壓，曲面橫軸換算成對應 ride height。

掃描預設每軸 21 點，FV / SV 共 441 個組合，解析度可設為 5–51 點。WE 除 caster × steering 網格外，另輸出目前設計 caster 的 steering 曲線。計算在背景執行；改變設定會取消過期結果。不可達姿態會保留錯誤原因，曲面會留空格，CSV 的 `valid` 與 `error` 可用來篩選。

## 單位、座標與計算定義

- JSON 長度一律 m、角度 deg、質量 kg、剛性 N/m；anti 與煞車比例採 0–1，`0.2` 表示 20%。UI 欄位標示各自單位。
- 靜態全車與 WE：X 向前、Y 向左、Z 向上。WE 顯示右輪 FR / RR，座標原點在前軸中心地面；後軸 X 約為負軸距。WE 正轉角為繞向上主銷的右手正向。
- FV：橫向 X 向車右，垂直座標在來源程式叫 Y、圖表與 CSV 以 Z 表示。正行程為輪側接地參考點相對車身向上。正 roll 為右接地點較高。
- FV camber 相對當下左右接地點連線；輪胎上端向外為正。`camber_gain_*_deg_per_mm` 是固定另一輪行程時，該輪 camber 對自身行程的局部偏導，包含地面基準改變；以 ±0.01 mm 中央差分求值。它不同於純 heave 曲線的總斜率。
- SV 來源 X 由前往後，前軸約為 -wheelbase、後軸為 0。正 pitch 為後接地點較高。`ride_height_*_mm = 靜態車底離地高 × 1000 − 對應輪側行程`；此處是 2D 姿態的等效高度，不包含完整車身姿態平衡。
- RC / PC 使用隨接地連線移動、旋轉的座標系：原點在當前兩接地點的中點。FV 的 x′ 由左輪指向右輪，SV 的 x′ 由前輪指向後輪；y′ 是 x′ 逆時針旋轉 90° 的向上法線。因此 y′ 就是到當前接地線的有號垂直高度，x′ 是沿線方向相對中點的位置。計算式為 `x′=(P−O)·u`、`y′=(P−O)·n`。
- 曲面、位置分布與主要數值使用 `rc_x_prime_mm` / `rc_y_prime_mm`、`pc_x_prime_mm` / `pc_y_prime_mm`；另以 `*_world_x_mm` / `*_world_z_mm` 保留固定車身座標作查核。舊版結果的 `rc_x_mm` / `rc_z_mm` 與 `pc_x_mm` / `pc_z_mm` 是固定座標，與新欄位的基準不同。
- 動態機構仍畫在固定車身 X / Z 畫面內，讓硬點運動與原始參考點容易比較；青色 x′ 軸沿接地線，橘色 y′ 軸隨之旋轉並保持垂直。幾何圖持續維持等比例、滑動時保留縮放與視角。
- FV 圓點標示：粉紅實心 RC、白色大空心 RC0、黃色 CG、青色 IC L、紫色 IC R；均附名稱。RC0 是當前專案零行程的原始 RC，CG 固定在車身上，兩者不隨輪端行程平移。零行程 RC 與 RC0 重疊時，仍可看見外圈與中心點。SV 同樣標示 PC / PC0、CG 與前後 IC。關閉力線不會隱藏座標軸或圓點；超出目前視野的 IC 可縮小或平移查看。
- `svsa` 是接地點至側視瞬心的水平距離，並非斜邊長。0 保留原 anti-dive / anti-lift 求交點；大於 0 時使用輸入 SVSA 與 anti-dive 決定 IC，anti-lift 由幾何算出，輸入的 anti-lift 不再約束 IC。
- `clearance_steer_deg` 是靜態輪圈／避讓構造的設計轉角（原值 36 deg）；與 WE 掃描的 `analysis.steer_deg`（原值 ±30 deg）分開保存。

## 模型適用範圍

FV 沿用雙圓求交的 2D 四連桿運動，車身上下接點固定，零位統一使用套件內靜態求解器與目前 JSON 專案。每次求解都以凍結零位為基準，防止多次滑動造成累積漂移；反算後會核對實際行程殘差。接點延長線平行時，RC / PC 顯示 N/A。

SV 沿用原 `sus_sv_v4.py` 的 **Au / Al 固定切線路徑**，維持羊角剛性，並以真實接地參考點高度反算行程；不是完整 3D A 臂懸吊運動。Caster 隨羊角姿態與接地線傾角重新計算。Anti 數值保留原 SV 設計公式：兩軸 anti-dive 均以前煞車比例正規化，anti-lift 用 1−前煞車比例；這是與來源幾何一致的設計指標，不能直接當成完整前後軸煞車 / 驅動受力比例。

WE 使用靜態全車解出的主銷軸與輪心做 Rodrigues 旋轉，不使用原 WE 腳本另外建立的主銷公式。固定主銷 caster 本身不隨 steering 改變；因此提供「不同設計 caster × steering → 輪軸傾角」曲面，以及固定設計 caster 的水平曲線。

WE 區分兩個點：`material_*` 是零轉角接地材料點隨輪胎剛體轉動的位置；`contact_*` 是旋轉後、以負載半徑建模的輪胎中心圓最低點。前者到固定主銷的垂距理應恆定，後者的選點改變才會產生半徑變化。圖中的輪胎外形保留自由半徑；未求解有限胎寬的胎肩接觸、彈性變形、轉向頂升平衡，也未將 FV / SV 行程與 steering 耦合成完整 3D 懸吊求解。

## 匯出

| 按鈕 | 內容 |
|---|---|
| 姿態 CSV | 目前姿態所有結果與單位欄位；靜態頁輸出命名硬點 |
| 硬點 CSV | 目前輪側接點、輪心、RC / PC、零位中心與 CG；FV / SV 額外含 `x_prime_m` / `y_prime_m` 接地座標。靜態頁含四角主銷、UBJ / LBJ、輪心與 A 臂車身接點。座標 m |
| 範圍 CSV | 全部網格點與不可達原因；尚未掃描時會自動計算。WE 另含 `steering_sweep` 行 |
| PNG / SVG | 目前視角與選定圖表；PNG 為 2 倍解析度，曲面保留色面 |
| HTML 報告 / PDF | 目前圖表、姿態、掃描有效筆數、模型說明與完整專案／操作快照。瀏覽器開啟 HTML 後可「列印 / 另存 PDF」 |

CSV 使用 UTF-8 BOM，可直接以 Excel 開啟。`*_mm`、`*_deg`、`*_pct`、`*_m` 與 `*_deg_per_mm` 表示欄位單位；`axis_x` / `axis_y` 是所選掃描座標，SV 行程模式為換算後前 / 後 ride height。曲面圖會標示座標與單位。

FV / SV 硬點 CSV 的 `x_m` / `z_m` 是固定車身座標，`x_prime_m` / `y_prime_m` 是當前接地中點座標。報告的輸入快照另存該姿態的接地座標原點與兩個單位基底；PNG / SVG 與報告都保留圓點與 x′ / y′ 軸。

## 驗證

2026-11-02，Windows / Python 3.12.14 / NumPy 2.3.5 / Pillow 12.3.0。包含原 Vehicle Lab 的完整原生介面測試 99 項通過。新增測試涵蓋靜態場景一致性、JSON 往返與錯誤拒絕、FV 連桿長度／鏡像／gain、SV 切線與剛體約束、SVSA、姿態反算、WE 主銷來源與材料點半徑不變性、不可達網格，以及全部圖表的 PNG / SVG / CSV / HTML 匯出。

```powershell
$env:VEHICLE_LAB_GUI_TEST = '1'
$env:SUSPENSION_LAB_GUI_TEST = '1'
python -m unittest discover -s tests -v
```

若受限執行環境的系統暫存目錄沒有寫入權限，可先將 `TEMP` 與 `TMP` 指向專案內已存在的 `.test-tmp` 目錄再執行測試。

2026-11-03 新增接地座標系與圓點標記驗證：核對座標基底正交、世界剛體旋轉／平移不變性、RC / PC 垂直高度、前後 FV 位置曲面、無有限 RC 處理，以及 PNG / SVG 圓點與硬點 CSV 的接地座標欄位。

2026-11-03 完成幾何模組內移：20 個靜態幾何／預覽模組與輪端幾何模組已收進本套件，移除 `new` 搜尋路徑與外部 import。包含原生介面的完整測試 **110 項通過，0 失敗、0 錯誤、0 略過**。新增隔離測試只複製 `suspension_lab` 與 `vehicle_lab`，在沒有 `new` 的目錄以 Python 隔離模式啟動，禁止匯入舊頂層套件，驗證專案往返、FV／SV／WE 求解與掃描、靜態圖表、GUI／預覽模組及命令列入口。搬移前後另比對預設與自訂專案的完整線框、硬點與多組姿態結果，序列化雜湊完全一致。
