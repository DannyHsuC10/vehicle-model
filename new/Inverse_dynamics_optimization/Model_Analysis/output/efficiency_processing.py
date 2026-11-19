# 從 CSV 中提取 _x / _y / _z 欄位，並輸出新的 CSV
from pathlib import Path
import pandas as pd
import glob

file_name = "output_20260519_015334.csv"
output_file_name = "position_output.csv"
# =========================
# 1. 用相對路徑找 CSV
# =========================
BASE_DIR = Path(__file__).resolve().parent
target_file = BASE_DIR / file_name

# 這裡先示範讀第一個 CSV
file_path = target_file

print(f"Reading: {file_path}")

df = pd.read_csv(file_path)

# =========================
# 2. 找出 _x / _y / _z 欄位
# =========================
pos_cols = [col for col in df.columns if col.endswith(("_x", "_y", "_z"))]

print("Position columns found:")
print(pos_cols)

# =========================
# 3. 建立新 CSV 並輸出
# =========================
output_df = df[pos_cols]

output_path = BASE_DIR / output_file_name
output_df.to_csv(output_path, index=False)

print(f"Saved to: {output_path}")
