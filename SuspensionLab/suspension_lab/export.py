"""Portable reports include the exact project, pose and mapping provenance."""
import csv
import datetime
import html
import json
from pathlib import Path

from .analysis import NOTES, PAGES
from .engine import to_contact_frame


def write_csv(path, rows):
    keys = list(dict.fromkeys(k for row in rows for k in row))
    with Path(path).open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def point_rows(state):
    rows = []
    for name, p in state["points"].items():
        row = {"name": name, **{axis+"_m": value for axis, value in zip("xyz" if len(p) == 3 else "xz", p)}}
        if "frame" in state and len(p) == 2:
            local = to_contact_frame(p, state["frame"])
            row.update(x_prime_m=float(local[0]), y_prime_m=float(local[1]))
        rows.append(row)
    return rows


def write_report(path, project, page, state, mapping, svg, controls, *, derived_parameters=None, mount_position_checks=None):
    esc = lambda value: html.escape(str(value))
    metrics = state.get("metrics", {}) if state else {}
    metric_table = "".join(f"<tr><th>{esc(k)}</th><td>{esc('N/A' if v is None else format(v, '.7g') if isinstance(v, (float, int)) else v)}</td></tr>" for k, v in metrics.items())
    coverage = f"範圍掃描：{mapping['valid']} / {mapping['total']} 筆有效。缺值代表不可達姿態或無有限交點。" if mapping else "本報告未包含範圍掃描。"
    snapshot = json.dumps({"project": project, "view_controls": controls,
                           "static_parameters": derived_parameters,
                           "mount_position_checks": mount_position_checks,
                           "contact_frame": (state or {}).get("frame")}, ensure_ascii=False, indent=2)
    method = ("硬點模式：以 24 個 XYZ 定義幾何；FV 使用等效樞點，SV 使用硬點軸線的固定切線近似，WE 使用 UBJ–LBJ 主銷。"
              "快照內 front／rear 是保留的參數模式設計輸入，static_parameters 才是目前硬點反算值。"
              if project.get("hardpoints", {}).get("enabled") else "參數模式：由前後軸設計參數解算靜態幾何。")
    content = f'''<!doctype html><html lang="zh-Hant"><meta charset="utf-8">
<title>Suspension Lab · {esc(PAGES[page])}</title><style>
body{{font:14px 'Microsoft JhengHei',sans-serif;margin:32px;color:#172b4d}}h1{{font-size:26px}}
table{{border-collapse:collapse}}th,td{{padding:5px 12px;border:1px solid #ccd5df;text-align:left}}
svg{{max-width:100%;height:auto}}pre{{white-space:pre-wrap;background:#f1f5f9;padding:16px}}
@media print{{button{{display:none}}svg,table{{break-inside:avoid}}}}
</style><button onclick="window.print()">列印 / 另存 PDF</button>
<h1>Suspension Lab · {esc(PAGES[page])}</h1><p>{esc(project['name'])} · {datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat(timespec='seconds')}</p>
<p>{esc(method)}</p><p>{esc(NOTES[page])}</p><p>{esc(coverage)}</p>{svg}<h2>目前姿態</h2><table>{metric_table}</table>
<h2>可重現輸入快照</h2><pre>{esc(snapshot)}</pre>
<p>完整範圍資料與命名硬點請使用介面底部的 CSV 匯出。曲面座標依圖示單位，硬點 CSV 為 m。</p></html>'''
    Path(path).write_text(content, encoding="utf-8")
