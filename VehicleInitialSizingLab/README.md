# Vehicle Initial Sizing Lab

車輛初步設計工作台。左側設定參數，右側顯示當次求解產生的 Markdown 報告，包含表格、公式、數值代入與完整結果索引。

## 使用方式

1. 雙擊 `Start-VehicleInitialSizingLab.bat`，開啟獨立桌面視窗。
2. 點左側上方的頁籤切換車輛、性能、懸吊、輪胎和馬達設定。拖曳中間的分隔線可調整輸入區寬度。煞車減速度留空時沿用 `ax`。
3. 按上方「儲存」或 `Ctrl+S` 保存參數專案，預設位置為本工具的 `projects/` 資料夾。按「匯入」或 `Ctrl+O` 載入 JSON，按「另存」保存另一份。
4. 按「求解」、`F5` 或 `Ctrl+Enter`。在右側「求解報告」頁籤查看 Markdown 表格及 LaTeX 公式。用「跳至章節」查看各項分析，或按右下角「儲存 Markdown」另存當次報告。專案 JSON 保存輸入參數，載入後可重新求解。
5. 關閉桌面視窗即可結束。

「匯出 CSV」位於右下角，會輸出當次求解的輸入參數和完整結果。欄位為 `section`、`field`、`value`，無可用數值的項目留空。CSV 使用 UTF-8 BOM，可用 Excel 開啟。

## 匯出至 SuspensionDynamicsLab

在左側 **Dynamics** 頁籤輸入目標阻尼比與五個車架參數。求解後，按右下角 **匯出 Dynamics JSON**。車輛資料集中在 `parameters.full`，要在 SuspensionDynamicsLab 開啟時，需先將資料填入完整專案。Dynamics 頁籤的設定可隨 Initial Sizing 專案儲存。舊專案開啟時會補上模板預設值。

- 前後每輪簧下質量採同一個 `mu`，前軸驅動比例為 0.5，前軸煞車比例採理想分配擬合值。四個 Anti 直接輸出求解比例。
- 前後 Heave 採純主簧，簧率和阻尼係數除以 4 轉為 Dynamics 的雙輪位移總和通道。前後 Roll 採用估算值。中央 Roll 與四角直推的剛性和阻尼為零。
- 目標阻尼比用於 Heave、Roll 和輪胎阻尼。輪胎各自使用 `Kt`，以 (2ζ√(Kt·mu)) 估算阻尼。車架扭轉與傳力阻尼採手動輸入。
- Tender 剛性保存在 `parameters.full.tender_stiffness_N_per_m`，無有效結果時為 `null`。Heave 匯出採主簧設定。
- 其餘全車欄位（例如 `Iyy`、Warp）沿用隨附的 `dynamics_template.json`。

## 預設參數

啟動、新增專案和恢復預設時，工具會讀取 [default_project.json](default_project.json) 的 `parameters`。可直接修改 JSON 裡的數值，或在介面將一組參數另存成這個檔案。將 `brake_ax` 設為 `null` 代表求解時沿用 `ax`。

## 首次安裝

需要 Windows x64、Python 3.10+（含 Tkinter）與 Microsoft Edge WebView2 Runtime。此電腦已建置檢視器，可直接啟動。

在其他電腦首次建置時，在本資料夾開啟 PowerShell。建置腳本會先使用 Visual Studio 隨附的 WebView2 SDK。若使用從 NuGet 解壓的 `Microsoft.Web.WebView2` SDK，可用 `-SdkRoot` 指定其資料夾。

```powershell
.\Build-WebView2Host.ps1
```

工具列、參數頁籤和章節選單使用 Tkinter／ttk 的 `clam` 主題。右側的 WebView2 讀取隨附的本機資源，Markdown 排版和 LaTeX 渲染可離線完成。建置產物位於 `build/webview2/`。

求解器沿用 [Initial settings](../new/Initial%20settings/vehicle_initial_sizing.py) 的公式與預設值。模型假設與推導見 [計算說明](../new/Initial%20settings/vehicle_initial_sizing_calculations.md)。

Markdown 與公式分別使用隨附的 Marked 17.0.5 與 MathJax 3.2.2 排版。授權檔在 `web/vendor/`。
