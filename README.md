# 車輛設計測試工具

各套工具放在獨立的子資料夾，可分開啟動或單獨複製到其他位置使用。

| 工具 | 啟動方式 | 說明 |
| --- | --- | --- |
| SuspensionGeometryLab：懸吊幾何工作台 | 雙擊 `SuspensionGeometryLab/Start-SuspensionGeometryLab.bat` | [操作說明](SuspensionGeometryLab/README.md) |
| VehicleLab：整車分析工具 | 雙擊 `VehicleLab/Start-VehicleLab.bat` | [操作說明](VehicleLab/README.md) |
| SuspensionDynamicsLab：懸吊彈簧阻尼模型分析 | 雙擊 `SuspensionDynamicsLab/Start-SuspensionDynamicsLab.bat` | [操作說明](SuspensionDynamicsLab/README.md) |
| VehicleInitialSizingLab：車輛初步設計與 Markdown 報告 | 雙擊 `VehicleInitialSizingLab/Start-VehicleInitialSizingLab.bat` | [操作說明](VehicleInitialSizingLab/README.md) |

`SuspensionGeometryLab/Start-StaticWireframe.bat` 是懸吊工具內的靜態線框預覽。

SuspensionGeometryLab、VehicleLab 和 SuspensionDynamicsLab 各有 `tests/` 與 `Start-Tests.bat`。雙擊該工具內的 `Start-Tests.bat` 會執行完整測試（含原生視窗），也可在該工具資料夾執行 `python -m unittest discover -s tests -v`。

各套工具皆需要 Python 3.10+ 與 Tkinter。VehicleInitialSizingLab 另以 WebView2 顯示 Markdown 和 LaTeX。SuspensionGeometryLab 與 SuspensionDynamicsLab 另需要 NumPy，PNG 匯出需要 Pillow。

既有車輛專案位於 `VehicleLab/projects/vehicle_project.json`；懸吊專案可存入 `SuspensionGeometryLab/projects/`。兩套工具的 JSON 格式不同，請各自開啟對應的專案。

新增的 `SuspensionDynamicsLab` 將 `new/Model_testing` 整合為四類、十三種彈簧阻尼模型：1/4、1/2 左右、1/2 前後、全車。可直接編輯參數、計算、比較曲線與播放 Canvas 動畫，也可開啟舊批次 PKL；新專案與結果分別儲存為 JSON 與 NPZ。與另兩套工具的專案格式不同。

`new/` 保留原有模型與研究內容；根目錄的 `.test-output/`、`.test-tmp/` 與日誌保留歷次測試資料。分裝前的舊壓縮檔移至 `VehicleLab/archive/VehicleLab-before-split.zip`。

2026-11-05 分裝驗證：SuspensionGeometryLab 89 項、VehicleLab 83 項測試全部通過（含原生介面），各自通過隔離複製驗證。3 組懸吊設計與整車 9 個工具的計算快照在分裝前後完全一致。完整測試紀錄位於 `.test-output/suspension-separated-tests.log` 與 `.test-output/vehicle-separated-tests.log`。

2026-11-05 懸吊動態工具驗證：SuspensionDynamicsLab 34 項測試通過，涵蓋 13 模型、63 組相容情境、38 組舊結果匯入與獨立複製啟動；詳見 [驗證紀錄](SuspensionDynamicsLab/VALIDATION.md)。
