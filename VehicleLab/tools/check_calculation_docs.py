"""Check local links and Markdown/LaTeX structure; build a source-function index.

This is a structural checker, not a TeX typesetter or a proof of the equations.
"""
import argparse
import ast
import json
from pathlib import Path
import re
from urllib.parse import unquote

ROOT=Path(__file__).resolve().parents[1]
DOCS=ROOT/"計算說明"


def chapter(prefix):
    matches=list(DOCS.glob(prefix+"_*.md"))
    if len(matches)!=1:raise ValueError(f"Expected one chapter {prefix}")
    return matches[0].name


def references(module,name):
    if module=="models":return ["12"] if name in ("write_project","read_project") else ["01"]
    if module=="physics":
        groups={"00":["linspace","interpolate"],"01":["frequency"],"02":["step_response","transmissibility"],
            "03":["damper_force","damper_curves"],"07":["roll_stiffness","distribution","lift_limit"],
            "08":["tire_peak","magic_force","slip_for_force","steady_balance"],
            "09":["air_density","wing_coefficients","aero"],"10":["rigid_transfer","contact_patch"]}
        return [key for key,names in groups.items() if name in names]
    if module=="linalg":return ["04"]
    if module=="geometry":
        if name in ("wheel_basis","wheel_state"):return ["04"]
        if name=="plane_line":return ["04","05"]
        if name in ("steering_metrics","ackermann_percent","set_kingpin_goals"):return ["06"]
        return ["05"]
    if module=="lap":return ["11"]
    if module=="motor":return ["11"]
    if module=="workflows":return ["00","12"]
    return ["12"]


def write_index():
    lines=["# 13　方程式與程式函式索引","",
        "此索引由 Python AST 讀取目前原始碼產生。表內連結指出函式所在檔案與詳細方程式章節；巢狀殘差/導數函式由其外層求解器章節說明。純 UI 排版與事件綁定不另建立物理方程式。","",
        "重建指令：`python tools/check_calculation_docs.py --write-index`。","" ]
    for module in ("models","physics","linalg","geometry","motor","lap","workflows","export","charts","cli"):
        path=ROOT/"vehicle_lab"/(module+".py")
        tree=ast.parse(path.read_text(encoding="utf-8"))
        lines.extend([f"## {module}.py","",f"原始碼：[{module}.py](../vehicle_lab/{module}.py)","",
            "| 函式 / 方法 | 原始碼行 | 方程式與流程 |","| --- | ---: | --- |"])
        for node in tree.body:
            functions=[(node.name,node)] if isinstance(node,(ast.FunctionDef,ast.AsyncFunctionDef)) else [
                (node.name+"."+f.name,f) for f in node.body if isinstance(f,ast.FunctionDef) and f.name!="__init__"] if isinstance(node,ast.ClassDef) else []
            for name,fn in functions:
                refs=references(module,fn.name)
                if not refs:raise ValueError(f"Unmapped calculation: {module}.{name}")
                links="、".join(f"[{r}]({chapter(r)})" for r in refs)
                lines.append(f"| `{name}` | {fn.lineno} | {links} |")
        lines.append("")
    lines.extend(["## GUI 的額外計算入口","","原始碼：[gui.py](../vehicle_lab/gui.py)","",
        "| 方法 | 詳細章節 |","| --- | --- |"])
    gui={"01":["RateEditor.refresh","RateEditor.apply","RateEditor.sync"],
        "05":["GeometryEditor.mirror","GeometryEditor.reset","ToolPanel.geometry_goals"],
        "06":["ToolPanel.kingpin_goals"],
        "09":["ToolPanel.density_helper","ToolPanel.apply_wing"],
        "11":["VehicleEditor.select_motor_csv","VehicleEditor.preview_motor_curve"],
        "12":["open_dialog","VehicleEditor.edit","VehicleEditor.parameter_dialog",
              "ToolPanel.calculate","ToolPanel.receive","ToolPanel.animate","ToolPanel.export",
              "App.open_vehicle_editor","App.open_geometry_editor","App.open_rate_editor","App.open_track_editor",
              "App.project_file_dialog","App.poll","App.destroy"]}
    for key,names in gui.items():
        for name in names:lines.append(f"| `{name}` | [{key}]({chapter(key)}) |")
    lines.extend(["","行號會隨程式修改而變動；重新執行索引工具即可更新。此表表示文件定位，不表示每個函式都是獨立物理模型。"])
    (DOCS/"13_方程式與程式函式索引.md").write_text("\n".join(lines)+"\n",encoding="utf-8")


def prose_only(text):
    output=[];fence=None
    for line in text.splitlines(keepends=True):
        match=re.match(r"\s*(```+|~~~+)",line)
        if match:
            marker=match.group(1)[0]
            if fence is None:fence=marker
            elif fence==marker:fence=None
            continue
        if fence is None:output.append(re.sub(r"`[^`\n]*`","",line))
    if fence is not None:raise ValueError("Unclosed fenced code block")
    return "".join(output)


def escaped(text,index):
    count=0;index-=1
    while index>=0 and text[index]=="\\":count+=1;index-=1
    return count%2==1


def math_blocks(text):
    blocks=[];i=0
    while i<len(text):
        if text[i]!="$" or escaped(text,i):i+=1;continue
        display=text.startswith("$$",i);width=2 if display else 1
        start=i+width;j=start
        while j<len(text):
            if text[j]=="$" and not escaped(text,j) and (not display or text.startswith("$$",j)):break
            j+=1
        if j==len(text):raise ValueError("Unclosed math delimiter")
        content=text[start:j]
        if not display and "\n" in content:raise ValueError("Inline math unexpectedly spans lines")
        blocks.append((display,content));i=j+width
    return blocks


def check_math(math):
    level=0
    for i,char in enumerate(math):
        if char not in "{}" or escaped(math,i):continue
        level+=1 if char=="{" else -1
        if level<0:raise ValueError("Unmatched closing brace in math")
    if level:raise ValueError("Unmatched opening brace in math")
    stack=[]
    for action,environment in re.findall(r"\\(begin|end)\{([^}]+)\}",math):
        if action=="begin":stack.append(environment)
        elif not stack or stack.pop()!=environment:raise ValueError("Mismatched math environment")
    if stack:raise ValueError("Unclosed math environment")
    if len(re.findall(r"\\left\b",math))!=len(re.findall(r"\\right\b",math)):
        raise ValueError("Mismatched left/right math delimiters")


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--write-index",action="store_true")
    parser.add_argument("--output",type=Path,default=ROOT/".test-output"/"calculation-docs"/"docs-audit.json")
    args=parser.parse_args()
    if args.write_index:write_index()
    errors=[];stats=[]
    for path in sorted(DOCS.glob("*.md")):
        raw=path.read_text(encoding="utf-8")
        try:
            text=prose_only(raw);blocks=math_blocks(text)
            for display,math in blocks:check_math(math)
            for target in re.findall(r"(?<!!)\[[^\]\n]+\]\(([^)\n]+)\)",text):
                if re.match(r"[a-zA-Z][a-zA-Z0-9+.-]*:",target) or target.startswith("#"):continue
                clean=unquote(target.split("#",1)[0]).strip("<>")
                if clean and not (path.parent/clean).exists():raise ValueError("Missing relative link: "+target)
            stats.append({"file":path.name,"lines":len(raw.splitlines()),"bytes":len(raw.encode("utf-8")),
                "display_equations":sum(display for display,_ in blocks),"inline_equations":sum(not display for display,_ in blocks)})
        except ValueError as exc:errors.append({"file":path.name,"error":str(exc)})
    report={"documents":len(stats),"total_lines":sum(s["lines"] for s in stats),
        "display_equations":sum(s["display_equations"] for s in stats),
        "inline_equations":sum(s["inline_equations"] for s in stats),"errors":errors,"files":stats,
        "scope":"Structural math, fenced-code, and relative-link check; not a TeX renderer or proof."}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps({key:value for key,value in report.items() if key!="files"},ensure_ascii=False))
    return bool(errors)


if __name__=="__main__":raise SystemExit(main())
