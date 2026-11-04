# 車輛設計測試工具

兩套工具已分裝成各自獨立的子資料夾，可分開啟動、測試，或單獨複製到其他位置使用。

| 工具 | 啟動方式 | 說明 |
| --- | --- | --- |
| SuspensionLab：懸吊幾何工作台 | 雙擊 `SuspensionLab/Start-SuspensionLab.bat` | [操作說明](SuspensionLab/README.md) |
| VehicleLab：整車分析工具 | 雙擊 `VehicleLab/Start-VehicleLab.bat` | [操作說明](VehicleLab/README.md) |

`SuspensionLab/Start-StaticWireframe.bat` 是懸吊工具內的靜態線框預覽。

每套工具都有自己的 `tests/` 與 `Start-Tests.bat`。雙擊該工具內的 `Start-Tests.bat` 會執行完整測試（含原生視窗）；也可切換到該工具資料夾後執行 `python -m unittest discover -s tests -v`。

兩套工具皆需要 Python 3.10+ 與 Tkinter。SuspensionLab 另需要 NumPy；PNG 匯出需要 Pillow，依各自操作說明安裝相應套件。

既有車輛專案位於 `VehicleLab/projects/vehicle_project.json`；懸吊專案可存入 `SuspensionLab/projects/`。兩套工具的 JSON 格式不同，請各自開啟對應的專案。

`new/` 保留原有模型與研究內容；根目錄的 `.test-output/`、`.test-tmp/` 與日誌保留歷次測試資料。分裝前的舊壓縮檔移至 `VehicleLab/archive/VehicleLab-before-split.zip`。

2026-11-05 分裝驗證：SuspensionLab 89 項、VehicleLab 83 項測試全部通過（含原生介面），各自通過隔離複製驗證。3 組懸吊設計與整車 9 個工具的計算快照在分裝前後完全一致。完整測試紀錄位於 `.test-output/suspension-separated-tests.log` 與 `.test-output/vehicle-separated-tests.log`。
