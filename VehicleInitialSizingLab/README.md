# Vehicle Initial Sizing Lab

車輛初步設計工作台。左側設定參數，右側顯示當次求解產生的 Markdown 報告，包含表格、公式、數值代入與完整結果索引。

## 使用方式

1. 雙擊 `Start-VehicleInitialSizingLab.bat`。工具會在預設瀏覽器開啟本機介面。也可以在此資料夾執行 `python run.py`。
2. 調整左側的車重、幾何和性能目標。懸吊、輪胎、空力和馬達設定按主題展開。煞車減速度留空時沿用 `ax`。
3. 按「求解」或 `Ctrl+Enter`。用右上角的「跳至章節」查看各項分析，或按「下載 Markdown」保存當次完整報告。
4. 使用完畢後，關閉啟動視窗，或在啟動視窗按 `Ctrl+C`。

需要 Python 3.10+ 與現代瀏覽器。計算、Markdown 排版及公式渲染在本機完成，可離線使用。

求解器沿用 [Initial settings](../new/Initial%20settings/vehicle_initial_sizing.py) 的公式與預設值。模型假設與推導見 [計算說明](../new/Initial%20settings/vehicle_initial_sizing_calculations.md)。

Markdown 與公式分別使用隨附的 Marked 17.0.5 與 MathJax 3.2.2 排版。授權檔在 `web/vendor/`。
