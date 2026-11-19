"""Export application-rendered chart examples; requires a graphical session."""
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from vehicle_lab.gui import App
from vehicle_lab.workflows import defaults,run_tool


def main():
    output=ROOT/"examples";output.mkdir(exist_ok=True)
    app=App();app.busy=True;app.update()
    try:
        for key in ("geometry","lap"):
            panel=app.panels[key];app.notebook.select(panel)
            result=run_tool(key,app.vehicle,app.geo,defaults(key),app.track)
            panel.result_vehicle=app.vehicle;panel.result_options=defaults(key)
            panel.receive(app.revision,result,None);app.busy=True;app.update()
            panel.chart.png(output/(key+"_preview.png"))
            (output/(key+"_preview.svg")).write_text(panel.chart.svg(),encoding="utf-8")
    finally:app.destroy()
    print("Exported previews to",output)


if __name__=="__main__":main()
