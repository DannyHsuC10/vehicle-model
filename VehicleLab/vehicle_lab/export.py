"""CSV, print-ready HTML reports and simple engineering CAD exports."""
import csv
import datetime
import html
import json
import math
from dataclasses import asdict
from pathlib import Path
from .linalg import sub, norm, unit, cross, scale, add


def write_csv(path,rows):
    keys=list(dict.fromkeys(k for r in rows for k in r))
    with Path(path).open("w",encoding="utf-8-sig",newline="") as handle:
        writer=csv.DictWriter(handle,fieldnames=keys)
        writer.writeheader(); writer.writerows(rows)


def write_report(path,title,vehicle,options,result,svg):
    esc=lambda x:html.escape(str(x))
    metrics="".join(f"<tr><th>{esc(k)}</th><td>{esc(v)}</td></tr>" for k,v in result["metrics"].items())
    params=esc(json.dumps({"vehicle":asdict(vehicle),"tool":options},ensure_ascii=False,indent=2))
    keys=list(dict.fromkeys(k for r in result["rows"] for k in r))
    head="".join(f"<th>{esc(k)}</th>" for k in keys)
    body="".join("<tr>"+"".join(f"<td>{esc(r.get(k,''))}</td>" for k in keys)+"</tr>" for r in result["rows"][:200])
    text=f'''<!doctype html><html lang="zh-Hant"><meta charset="utf-8"><title>{esc(title)}</title>
    <style>body{{font:14px 'Microsoft JhengHei',sans-serif;margin:30px;color:#182532}}h1{{font-size:26px}}
    table{{border-collapse:collapse;width:100%;font-size:11px}}th,td{{border:1px solid #ccd5df;padding:5px;text-align:left}}
    svg{{max-width:100%;height:auto}}pre{{white-space:pre-wrap;font-size:10px}}@media print{{button{{display:none}}svg{{break-inside:avoid}}}}
    </style><button onclick="window.print()">列印 / 另存 PDF</button><h1>Vehicle Lab · {esc(title)}</h1>
    <p>{esc(vehicle.name)} · {datetime.datetime.now().isoformat(timespec='seconds')}</p>
    <p>{esc(result['note'])}</p><table>{metrics}</table>{svg}<h2>輸入快照</h2><pre>{params}</pre>
    <h2>結果（最多 200 列；完整資料請匯出 CSV）</h2><table><tr>{head}</tr>{body}</table></html>'''
    Path(path).write_text(text,encoding="utf-8")


def write_stl(path,segments,radius=.008,sides=12):
    """Closed cylinders representing links; millimetres. Not production solids."""
    triangles=[]
    for a,b,kind in segments:
        if kind=="wheel": continue
        axis=sub(b,a)
        if norm(axis)<1e-8: continue
        axis=unit(axis)
        first=unit(cross(axis,[1,0,0] if abs(axis[0])<.8 else [0,1,0]))
        second=cross(axis,first)
        ring=lambda c:[add(c,scale(add(scale(first,math.cos(t*2*math.pi/sides)),scale(second,math.sin(t*2*math.pi/sides))),radius)) for t in range(sides)]
        ra,rb=ring(a),ring(b)
        for i in range(sides):
            j=(i+1)%sides
            triangles.extend([(ra[i],ra[j],rb[j]),(ra[i],rb[j],rb[i]),(a,ra[j],ra[i]),(b,rb[i],rb[j])])
    lines=["solid vehicle_lab_links"]
    for a,b,c in triangles:
        normal=unit(cross(sub(b,a),sub(c,a)))
        lines.append(" facet normal "+" ".join(f"{x:.9g}" for x in normal))
        lines.append("  outer loop")
        for point in (a,b,c): lines.append("   vertex "+" ".join(f"{x*1000:.9g}" for x in point))
        lines.extend(["  endloop"," endfacet"])
    lines.append("endsolid vehicle_lab_links")
    Path(path).write_text("\n".join(lines),encoding="ascii")


def write_step(path,segments):
    """ISO 10303-21 geometric curve set (wireframe, SI millimetres)."""
    entities=[]
    def entity(text):
        entities.append(text)
        return f"#{len(entities)}"
    application=entity("APPLICATION_CONTEXT('automotive_design')")
    entity(f"APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,{application})")
    context=entity(f"PRODUCT_CONTEXT('',{application},'mechanical')")
    product=entity(f"PRODUCT('Vehicle Lab','Suspension wireframe','',({context}))")
    formation=entity(f"PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE('','',{product},.NOT_KNOWN.)")
    dc=entity(f"PRODUCT_DEFINITION_CONTEXT('part definition',{application},'design')")
    definition=entity(f"PRODUCT_DEFINITION('design','',{formation},{dc})")
    shape=entity(f"PRODUCT_DEFINITION_SHAPE('','',{definition})")
    mm=entity("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))")
    angle=entity("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))")
    solid=entity("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())")
    uncertainty=entity(f"UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(0.000001),{mm},'distance_accuracy_value','')")
    rc=entity(f"(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT(({uncertainty}))GLOBAL_UNIT_ASSIGNED_CONTEXT(({mm},{angle},{solid}))REPRESENTATION_CONTEXT('','3D'))")
    curves=[]
    for a,b,kind in segments:
        refs=[]
        for point in (a,b):
            numbers=','.join(f'{x*1000:.9f}' for x in point)
            refs.append(entity(f"CARTESIAN_POINT('',({numbers}))"))
        curves.append(entity(f"POLYLINE('{kind}',({','.join(refs)}))"))
    curve_set=entity(f"GEOMETRIC_CURVE_SET('',({','.join(curves)}))")
    representation=entity(f"GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION('',({curve_set}),{rc})")
    entity(f"SHAPE_DEFINITION_REPRESENTATION({shape},{representation})")
    data="\n".join(f"#{i+1}={line};" for i,line in enumerate(entities))
    text=f"ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('Vehicle Lab wireframe, not manufacturing CAD'),'2;1');\nFILE_NAME('suspension.step','',(''),(''),'Vehicle Lab','Vehicle Lab','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n{data}\nENDSEC;\nEND-ISO-10303-21;\n"
    Path(path).write_text(text,encoding="ascii")
