"""Portable reports include the exact project, pose and mapping provenance."""
import datetime
import html
import json
from pathlib import Path

from vehicle_lab.export import write_csv
from .analysis import NOTES, PAGES
from .engine import to_contact_frame


def point_rows(state):
    rows = []
    for name, p in state["points"].items():
        row = {"name": name, **{axis+"_m": value for axis, value in zip("xyz" if len(p) == 3 else "xz", p)}}
        if "frame" in state and len(p) == 2:
            local = to_contact_frame(p, state["frame"])
            row.update(x_prime_m=float(local[0]), y_prime_m=float(local[1]))
        rows.append(row)
    return rows


def write_report(path, project, page, state, mapping, svg, controls):
    esc = lambda value: html.escape(str(value))
    metrics = state.get("metrics", {}) if state else {}
    metric_table = "".join(f"<tr><th>{esc(k)}</th><td>{esc('N/A' if v is None else format(v, '.7g') if isinstance(v, (float, int)) else v)}</td></tr>" for k, v in metrics.items())
    coverage = f"範圍掃描：{mapping['valid']} / {mapping['total']} 筆有效。缺值代表不可達姿態或無有限交點。" if mapping else "本報告未包含範圍掃描。"
    snapshot = json.dumps({"project": project, "view_controls": controls,
                           "contact_frame": (state or {}).get("frame")}, ensure_ascii=False, indent=2)
    content = f'''<!doctype html><html lang="zh-Hant"><meta charset="utf-8">
<title>Suspension Lab · {esc(PAGES[page])}</title><style>
body{{font:14px 'Microsoft JhengHei',sans-serif;margin:32px;color:#172b4d}}h1{{font-size:26px}}
table{{border-collapse:collapse}}th,td{{padding:5px 12px;border:1px solid #ccd5df;text-align:left}}
svg{{max-width:100%;height:auto}}pre{{white-space:pre-wrap;background:#f1f5f9;padding:16px}}
@media print{{button{{display:none}}svg,table{{break-inside:avoid}}}}
</style><button onclick="window.print()">列印 / 另存 PDF</button>
<h1>Suspension Lab · {esc(PAGES[page])}</h1><p>{esc(project['name'])} · {datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat(timespec='seconds')}</p>
<p>{esc(NOTES[page])}</p><p>{esc(coverage)}</p>{svg}<h2>目前姿態</h2><table>{metric_table}</table>
<h2>可重現輸入快照</h2><pre>{esc(snapshot)}</pre>
<p>完整範圍資料與命名硬點請使用介面底部的 CSV 匯出。曲面座標依圖示單位，硬點 CSV 為 m。</p></html>'''
    Path(path).write_text(content, encoding="utf-8")
