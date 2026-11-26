# Vehicle 14DOF Lab 網頁版

網頁使用原本的 Python 14 DOF 模型和 NumPy。Pyodide 在 Web Worker 中執行 9.125 m 定圓、75 m 直線加速及自訂賽道圈速。計算與匯入檔案留在瀏覽器內。首次執行需要連線至 jsDelivr 下載固定版本的 Pyodide 與 NumPy。

## GitHub Pages

1. 將本次變更推送到 GitHub 儲存庫。
2. 在 GitHub 的 **Settings → Pages → Build and deployment → Source** 選擇 **GitHub Actions**。
3. 在 **Actions → Deploy Vehicle14DOF web** 選擇 **Run workflow**，使用 `main` 分支。之後 `main` 上的相關程式變更會自動建置與部署。
4. 部署完成後從該次工作流程的 `github-pages` 環境開啟網址。

工作流程部署 `dist/vehicle14dof/`，內容包含靜態頁面、求解器原始碼、LP03 預設輸入、定圓範本與 Pacejka 授權。瀏覽器取得這些部署檔案的權限與網站一致。桌面版 FSAE 賽道、STL、遙測歷史與測試資料保留在儲存庫。

## 本機預覽

在儲存庫根目錄執行以下指令，再開啟 <http://127.0.0.1:8000/>。Windows 若已經有專案虛擬環境，可將 `python` 改成 `.\.venv\Scripts\python.exe`。

```powershell
python -m pip install -r Vehicle14DOFLab/requirements.txt
python -B web/build.py
python -m http.server 8000 --bind 127.0.0.1 --directory dist/vehicle14dof
```

## 輸入與結果

- 內建定圓半徑為 9.125 m，直線距離固定為 75 m。自訂工況可匯入封閉 LINE／ARC 賽道 CSV。頁面提供可直接匯入的定圓範例，內容為兩段相接的 180° 圓弧。座標、半徑與長度用公尺，起訖角度用度，方向填 `Clockwise` 或 `Counterclockwise`。每段終點須與下一段起點相接。
- 「車輛設定」可調整車身與重心、動力與空力、四輪、液壓煞車及懸吊，也可匯入桌面版詳細懸吊 JSON。車輛 JSON 引用的懸吊、輪胎、馬達及空力檔案可用各自的欄位提供。頁面上的值優先於匯入檔，留空則沿用匯入檔或 LP03 預設。
- 「輪胎與輪端」的自轉慣量、胎壓、垂向剛性與垂向阻尼由四輪共用。自由半徑、簧下質量、靜態外傾與驅動倍率按前後軸設定，同軸左右對稱。匯入設定若有差異，四輪共用值採左前輪，其餘輪端數值、Camber Gain 與直推通道採各軸左側。結果 ZIP 保留實際採用的值。
- 「懸吊與幾何」可設定前後軸 Roll Center 離地高度（mm）、四種 Anti（%）與前後軸 Camber Gain（每 25 mm 壓縮的角度變化）。Roll Center 與 Anti 需啟用傳力，Camber Gain 可單獨生效。匯入絕對值查表時，對應欄位會停用。直推按前後軸調整，同軸左右共用。各通道可設定固定運動比，或留空保留原曲線並選填運動比倍率。
- 「匯入設定與曲線」的五個欄位都附有可直接匯入的 LP03 範例。馬達範例使用 `vdc_V,rpm,T_shaft_Nm` 欄位表示多電壓包絡，扭矩為減速前馬達軸端的 N·m。空力係數填在 JSON 最外層，必填 `CL0` 和 `CD0`。
- 完成後進入駕駛回放。網頁以程式繪製賽車模型，根據遙測更新車身姿態、輪胎轉動與轉向，並同步顯示 G-G 加速度圓、油門、煞車和車速。支援跟車、俯視、0.25× 至 4× 倍速及時間軸。
- 速度著色軌跡、逐輪遙測與計時可在回放下方查看。完整 ZIP 包含設定快照、使用的輸入、逐點遙測 CSV、計時 CSV 與 SVG。
