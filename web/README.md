# Vehicle 14DOF Lab 網頁版

網頁使用原本的 Python 14 DOF 模型和 NumPy。Pyodide 在 Web Worker 中執行圈速、9.125 m 定圓及直線加速。計算與匯入檔案留在瀏覽器內。首次執行需要連線至 jsDelivr 下載固定版本的 Pyodide 與 NumPy。

## GitHub Pages

1. 將本次變更推送到 GitHub 儲存庫。
2. 在 GitHub 的 **Settings → Pages → Build and deployment → Source** 選擇 **GitHub Actions**。
3. 在 **Actions → Deploy Vehicle14DOF web** 選擇 **Run workflow**，使用 `main` 分支。之後 `main` 上的相關程式變更會自動建置與部署。
4. 部署完成後從該次工作流程的 `github-pages` 環境開啟網址。

工作流程只部署 `dist/vehicle14dof/`，內容包含靜態頁面、求解器原始碼、LP03 預設輸入與 Pacejka 授權。瀏覽器取得這些部署檔案的權限與網站一致。部署包不包含桌面版 STL、遙測歷史或測試資料。

## 本機預覽

在儲存庫根目錄執行以下指令，再開啟 <http://127.0.0.1:8000/>。Windows 若已經有專案虛擬環境，可將 `python` 改成 `.\.venv\Scripts\python.exe`。

```powershell
python -m pip install -r Vehicle14DOFLab/requirements.txt
python -B web/build.py
python -m http.server 8000 --bind 127.0.0.1 --directory dist/vehicle14dof
```

## 輸入與結果

- 工況頁可設定起步、車速目標、側向加速度目標、暖胎及計時圈數、賽道寬度或直線距離。
- 展開「車輛參數與輸入檔」即可匯入桌面版車輛設定 JSON 及懸吊專案 JSON。車輛檔內的懸吊、TIR、馬達或空力檔案路徑需用網頁各自的檔案欄位提供。未匯入的主要輸入會採用 LP03 預設檔，空力也可直接在車輛 JSON 中提供內嵌係數。質量、電壓與減速比欄位優先於匯入檔。
- 圈速支援自訂 LINE／ARC 賽道 CSV。執行完成後可查看速度著色的軌跡、俯視回放、逐輪遙測與計時。完整 ZIP 包含設定快照、使用的輸入、逐點遙測 CSV、計時 CSV 與 SVG。
- 桌面版逐項懸吊編輯器與 3D STL 回放目前僅在桌面版提供。網頁可載入既有的詳細設定，以 2D 方式回放當次結果。
