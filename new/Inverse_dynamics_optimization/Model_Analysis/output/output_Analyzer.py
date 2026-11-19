import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent

class MultiCSVAnalyzer:
    def __init__(self, file_list):
        self.data = {}

        for file in file_list:
            path = BASE_DIR / file  # 相對路徑<< 如果相對性要改改這裡

            name = Path(file).stem  # 用檔名當 case 名稱
            df = pd.read_csv(path)

            if "t" not in df.columns:
                raise ValueError(f"{file} 缺少 t 欄位")

            self.data[name] = {
                "df": df,
                "t": df["t"]
            }

        self.case_names = list(self.data.keys())

    def print_cases(self):    # 印所有 case
        print("\n===== CASE LIST =====")
        for name in self.case_names:
            print(name)

    def print_columns(self):#印欄位
        print("\n===== COLUMNS (from first case) =====")
        first = self.case_names[0]
        for col in self.data[first]["df"].columns:
            print(col)

    def preview(self, case=None, n=5):# preview
        if case is None:
            case = self.case_names[0]

        print(f"\n===== PREVIEW: {case} =====")
        print(self.data[case]["df"].head(n))

    def compare_time(self, col):# 單欄位多 case 比較
        plt.figure()

        for name in self.case_names:
            df = self.data[name]["df"]
            t = self.data[name]["t"]

            if col not in df.columns:
                print(f"[Warning] {col} 不在 {name}")
                continue

            plt.plot(t, df[col], label=name)

        plt.xlabel("Time (s)")
        plt.ylabel(col)
        plt.title(f"{col} Comparison")
        plt.legend()
        plt.grid()
        plt.show()

    def compare_xy(self, x_col, y_col):    # XY 比較（多 case）
        plt.figure()

        for name in self.case_names:
            df = self.data[name]["df"]

            if x_col not in df.columns or y_col not in df.columns:
                print(f"[Warning] 欄位不存在於 {name}")
                continue

            plt.plot(df[x_col], df[y_col], label=name)

        plt.xlabel(x_col)
        plt.ylabel(y_col)
        plt.title(f"{y_col} vs {x_col}")
        plt.legend()
        plt.grid()
        plt.show()

    def stats(self, col):# 統計比較
        print(f"\n===== STATS: {col} =====")

        for name in self.case_names:
            df = self.data[name]["df"]

            if col not in df.columns:
                print(f"{name}: 欄位不存在")
                continue

            print(f"{name}: min={df[col].min():.4f}, max={df[col].max():.4f}, mean={df[col].mean():.4f}")


if __name__ == "__main__":
    """    
    files = [
    "output_力量+阿克曼+角度優化.csv",
    "output_未優化.csv",
    "output_純阿克曼優化.csv"
    ]
    """
    files = ["output_20260511_002056.csv"]
    analyzer = MultiCSVAnalyzer(files)

    # 檢查
    analyzer.print_cases()# 印出有的檔案
    analyzer.print_columns()# 印出有的欄位
    #analyzer.preview()# 預覽一下

    # 單欄位比較(對時間函數)
    analyzer.compare_time("uAtie_thetaz")

    # 兩欄位函數
    analyzer.compare_xy("rack_y", "tie_rod_f")

    # 統計
    analyzer.stats("tie_rod_f")
