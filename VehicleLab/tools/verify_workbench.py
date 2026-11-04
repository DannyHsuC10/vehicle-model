"""Reproducible unit/UI and feature-matrix verification for the calculation manual."""
import argparse
from datetime import datetime,timezone,timedelta
import hashlib
import io
import json
import os
from pathlib import Path
import platform
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--gui",action="store_true")
    parser.add_argument("--output",type=Path,default=ROOT/".test-output"/"calculation-docs")
    args=parser.parse_args()
    if args.gui:os.environ["VEHICLE_LAB_GUI_TEST"]="1"
    args.output.mkdir(parents=True,exist_ok=True)
    log=io.StringIO()
    suite=unittest.defaultTestLoader.discover(str(ROOT/"tests"))
    tests=unittest.TextTestRunner(stream=log,verbosity=2).run(suite)
    (args.output/"tests.txt").write_text(log.getvalue(),encoding="utf-8")
    from vehicle_lab.models import Vehicle,read_project
    from vehicle_lab.geometry import default_geometry
    from vehicle_lab.workflows import NAMES,defaults,run_tool,rc_values
    from vehicle_lab.lap import PRESETS,sensitivities
    matrix=[]
    def execute(label,v,g,settings,track=None):
        for key in NAMES:
            try:
                out=run_tool(key,v,g,{**defaults(key),**settings.get(key,{})},track)
                assert out["rows"] and out["plots"],"Empty result"
                matrix.append({"case":label,"tool":key,"status":"pass","rows":len(out["rows"]),"plots":len(out["plots"])})
            except Exception as exc:
                matrix.append({"case":label,"tool":key,"status":"fail","error":str(exc)})
    for filename in ("fsae_example.json","fsae_no_aero.json"):
        v,g,s=read_project(ROOT/"examples"/filename)
        execute(filename,v,g,s,s.get("track"))
    from dataclasses import replace
    for front_ratio,rear_ratio in ((10.,12.),(12.,10.),(12.,12.),(14.,14.)):
        v=replace(Vehicle(),reduction_front=front_ratio,reduction_rear=rear_ratio)
        execute(f"FSAE_reduction_F{front_ratio:g}_R{rear_ratio:g}",v,default_geometry(v),{})
    v,g,s=read_project(ROOT/"examples"/"fsae_example.json")
    rf,rr=rc_values(v,g,s["lap"])
    for name in ("定圓 9.125 m","直線 75 m（開放起點）"):
        try:
            options={**defaults("lap"),"closed":"開放" not in name}
            out=run_tool("lap",v,g,options,PRESETS[name])
            assert out["rows"] and out["plots"],"Empty result"
            matrix.append({"case":name,"tool":"lap","status":"pass","rows":len(out["rows"]),
                           "plots":len(out["plots"]),"distance_m":out["metrics"]["distance_m"]})
        except Exception as exc:matrix.append({"case":name,"tool":"lap","status":"fail","error":str(exc)})
    try:
        rows=sensitivities(v,s["track"],rf,rr,step=10)
        assert len(rows)==9,"Expected baseline plus 8 comparisons"
        matrix.append({"case":"sensitivity","tool":"lap","status":"pass","rows":len(rows)})
    except Exception as exc:matrix.append({"case":"sensitivity","tool":"lap","status":"fail","error":str(exc)})
    hashes={str(p.relative_to(ROOT)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted((ROOT/"vehicle_lab").glob("*.py"))}
    from vehicle_lab.motor import read_motor_curve
    curve=read_motor_curve(v.motor_curve_csv)
    report={"timestamp_taipei":datetime.now(timezone(timedelta(hours=8))).isoformat(),
        "python":sys.version,"platform":platform.platform(),"gui_enabled":args.gui,
        "unit_tests":{"run":tests.testsRun,"failures":len(tests.failures),"errors":len(tests.errors),"skipped":len(tests.skipped)},
        "matrix":matrix,"source_sha256":hashes,
        "motor_csv":{"source":curve.source,"sha256":curve.sha256,"rows":len(curve.rpm),
                     "max_rpm":curve.max_rpm,"negative_samples":sum(x<0 for x in curve.torque)}}
    (args.output/"verification.json").write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")
    failures=[row for row in matrix if row["status"]!="pass"]
    print(json.dumps({"unit_tests":report["unit_tests"],"matrix_cases":len(matrix),"matrix_failures":failures,"output":str(args.output)},ensure_ascii=False))
    return 0 if tests.wasSuccessful() and not failures else 1


if __name__=="__main__":raise SystemExit(main())
