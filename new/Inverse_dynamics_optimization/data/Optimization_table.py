# 最佳化表格整理工具

import pandas as pd
from pathlib import Path
FILTER_DIR = Path(__file__).resolve().parent


geo_df = pd.read_csv(FILTER_DIR / "nodes.csv")
opt_keys = ["tie_rod"]

def generate_design_variables(geo_df, opt_keys):# df 產生
    rows = []

    for _, row in geo_df.iterrows():
        name = row["name"]

        if name not in opt_keys:
            continue

        for axis in ["x", "y", "z"]:
            val = row[axis]

            rows.append({
                "name": name,
                "axis": axis,
                "lower": val,
                "upper": val,
                "initial": val
            })

    return pd.DataFrame(rows)

def apply_range(df, delta=5):# 固定偏移
    df["lower"] = df["initial"] - delta
    df["upper"] = df["initial"] + delta
    return df

def apply_axis_range(df,axis_range):# 軸向偏移
    for axis, delta in axis_range.items():
        mask = df["axis"] == axis
        df.loc[mask, "lower"] = df.loc[mask, "initial"] - delta
        df.loc[mask, "upper"] = df.loc[mask, "initial"] + delta
    return df

def apply_custom_range(df):# 自定義偏移
    for i, row in df.iterrows():
        name = row["name"]
        axis = row["axis"]

        delta = custom_range[name][axis]

        df.at[i, "lower"] = row["initial"] - delta
        df.at[i, "upper"] = row["initial"] + delta

    return df

def save_csv(df, filename):#存檔
    path =Path(FILTER_DIR / filename)

    df.to_csv(path, index=False)
    print(f"Saved to {path}")


axis_range = {# 不同軸定義
    "x": 10,
    "y": 10,
    "z": 5
}

custom_range = {# 自定義區
    "tie_rod": {"x":5, "y":10, "z":3},
    "rack": {"x":2, "y":5, "z":1}
}


# 生成設計變數表格========================================
dv_df = generate_design_variables(geo_df, opt_keys)
dv_df = apply_range(dv_df, delta=5)# 全部都用初始數值偏移+-5

#dv_df = apply_axis_range(dv_df, axis_range)# xyz用不同的偏移
#dv_df = apply_custom_range(dv_df)# 按照自訂定義區定義
print(dv_df)# 印出
save_csv(dv_df, "design_vars.csv")
