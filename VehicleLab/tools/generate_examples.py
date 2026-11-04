"""Rebuild example projects without a desktop display."""
import copy
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from vehicle_lab.models import Vehicle,write_project
from dataclasses import replace
from vehicle_lab.geometry import default_geometry
from vehicle_lab.workflows import NAMES,defaults
from vehicle_lab.lap import PRESETS
from vehicle_lab.export import write_csv


def main():
    output=ROOT/"examples";output.mkdir(exist_ok=True)
    fsae=Vehicle()
    settings={key:defaults(key) for key in NAMES}
    settings.update(track=copy.deepcopy(PRESETS["示範 Autocross"]),track_name="示範 Autocross")
    for front,prefix in ((True,""),(False,"b_")):
        axle="front" if front else "rear"
        for short,long in (("c","comp"),("r","reb")):
            high=getattr(fsae,"damping_"+long+"_"+axle)*fsae.ratio(front)**2
            settings["damper"][prefix+short+"_high"]=high
            settings["damper"][prefix+short+"_low"]=5*high
    write_project(output/"fsae_example.json",fsae,default_geometry(fsae),settings)
    no_aero=replace(fsae,name="FSAE EV - no-downforce comparison (estimated)",cla_front=0.,cla_rear=0.)
    write_project(output/"fsae_no_aero.json",no_aero,default_geometry(no_aero),settings)
    write_csv(output/"track_segments.csv",PRESETS["示範 Autocross"])
    write_csv(output/"track_circle_9_125m.csv",PRESETS["定圓 9.125 m"])
    write_csv(output/"track_straight_75m.csv",PRESETS["直線 75 m（開放起點）"])
    geo=default_geometry(fsae)
    write_csv(output/"hardpoints.csv",[{"group":group,"corner":corner,"point":key,
        "x_mm":point[0]*1000,"y_mm":point[1]*1000,"z_mm":point[2]*1000}
        for group in ("corners",) for corner,points in geo[group].items() for key,point in points.items()])
    print("Created examples in",output)


if __name__=="__main__":main()
