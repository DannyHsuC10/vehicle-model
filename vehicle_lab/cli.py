"""Headless batch interface for reproducible numerical runs."""
import argparse
import json
from pathlib import Path
from .models import Vehicle,read_project
from .geometry import default_geometry
from .workflows import NAMES,defaults,run_tool
from .export import write_csv


def main():
    parser=argparse.ArgumentParser(description="Vehicle Lab offline numerical CLI")
    parser.add_argument("tool",choices=list(NAMES)+["all"])
    parser.add_argument("--project",type=Path)
    parser.add_argument("--output",type=Path,default=Path("results"))
    parser.add_argument("--set",action="append",default=[],metavar="KEY=JSON")
    args=parser.parse_args()
    try:
        if args.project:v,g,settings=read_project(args.project)
        else:v=Vehicle();g=default_geometry(v);settings={}
        overrides={}
        for item in args.set:
            key,value=item.split("=",1);overrides[key]=json.loads(value)
        args.output.mkdir(parents=True,exist_ok=True)
        for key in NAMES if args.tool=="all" else [args.tool]:
            options={**defaults(key),**settings.get(key,{}),**overrides}
            result=run_tool(key,v,g,options,settings.get("track"))
            write_csv(args.output/(key+".csv"),result["rows"])
            (args.output/(key+"_summary.json")).write_text(json.dumps({"metrics":result["metrics"],"options":options,
                "model":result["note"]},ensure_ascii=False,indent=2),encoding="utf-8")
            for index,plot in enumerate(result["plots"]):
                rows=[{"curve":s["name"],"x":x,"y":y,"x_label":plot["xlabel"],"y_label":plot["ylabel"]}
                    for s in plot["series"] for x,y in zip(s["x"],s["y"])]
                write_csv(args.output/(key+f"_curve_{index+1}.csv"),rows)
            print(key+": "+json.dumps(result["metrics"],ensure_ascii=False))
    except (ValueError,KeyError,TypeError,OSError) as exc:parser.exit(2,f"Error: {exc}\n")


if __name__=="__main__":main()
