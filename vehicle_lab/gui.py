"""Native desktop application. Tkinter is included with standard Python."""
import copy
import csv
import io
import json
import math
from dataclasses import asdict, fields, replace
from pathlib import Path
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, simpledialog

from .models import Vehicle, CORNERS, read_project, write_project
from . import geometry as geom
from . import physics
from .workflows import FIELDS, NAMES, NOTES, defaults, run_tool, rc_values
from .charts import Chart
from .export import write_csv, write_report, write_stl, write_step
from .lap import PRESETS, read_track, track_from_centerline, sensitivities
from .motor import read_motor_curve


LABELS={
    "name":"車輛名稱", "curb_mass":"整備質量（含電池）kg", "occupants":"駕駛與裝備 kg",
    "wheelbase":"軸距 m", "track_front":"前輪距 m", "track_rear":"後輪距 m", "cg_height":"總車重心高 m",
    "front_fraction":"前軸質量比例 0..1", "unsprung_front":"前輪簧下質量 kg/輪（左右相同）",
    "unsprung_rear":"後輪簧下質量 kg/輪（左右相同）", "unsprung_height_front":"前簧下重心高 m",
    "unsprung_height_rear":"後簧下重心高 m", "wheel_rate_front":"前輪端剛性 N/m", "wheel_rate_rear":"後輪端剛性 N/m",
    "motion_ratio_front":"前運動比 輪/彈簧", "motion_ratio_rear":"後運動比 輪/彈簧",
    "bell_ratio_front":"前搖臂比 輸入臂/輸出臂", "bell_ratio_rear":"後搖臂比 輸入臂/輸出臂",
    "damping_comp_front":"前壓縮阻尼 輪端 N s/m", "damping_reb_front":"前回彈阻尼 輪端 N s/m",
    "damping_comp_rear":"後壓縮阻尼 輪端 N s/m", "damping_reb_rear":"後回彈阻尼 輪端 N s/m",
    "arb_front":"前防傾桿 N m/rad", "arb_rear":"後防傾桿 N m/rad", "camber_front":"前靜態 camber deg",
    "arb_motion_ratio_front":"前防傾桿運動比 輪/桿臂", "arb_motion_ratio_rear":"後防傾桿運動比 輪/桿臂",
    "camber_rear":"後靜態 camber deg", "toe_front":"前靜態 toe-in deg/輪", "toe_rear":"後靜態 toe-in deg/輪",
    "cd":"車身 Cd", "frontal_area":"正面面積 m²", "cla_front":"前車身 ClA m²", "cla_rear":"後車身 ClA m²",
    "rho":"空氣密度 kg/m³", "tire_diameter_front":"前輪外徑 m", "tire_diameter_rear":"後輪外徑 m",
    "tire_width_front":"前胎寬 m", "tire_width_rear":"後胎寬 m", "pressure_front":"前胎壓 Pa", "pressure_rear":"後胎壓 Pa",
    "mu_lat":"橫向摩擦係數 μ", "mu_long":"縱向摩擦係數 μ", "load_sensitivity":"載重敏感性（2倍負載損失比例）",
    "camber_sensitivity":"外傾敏感性 每絕對 deg", "reference_tire_load":"輪胎 μ 參考負載 N",
    "tire_b":"Magic Formula B", "tire_c":"Magic Formula C", "tire_e":"Magic Formula E",
    "brake_front":"液壓前煞車比例 0..1", "rolling_coefficient":"滾阻係數",
    "motor_curve_csv":"單顆馬達 CSV（speed=RPM, torque=Nm）",
    "reduction_front":"前軸固定減速比（兩顆相同）",
    "reduction_rear":"後軸固定減速比（兩顆相同）",
    "reduction_efficiency_front":"前減速機效率 0..1",
    "reduction_efficiency_rear":"後減速機效率 0..1"}


def format_value(value):
    if value is None: return "—"
    if isinstance(value,float): return f"{value:.7g}"
    if isinstance(value,(list,dict)): return json.dumps(value,ensure_ascii=False)
    return str(value)


def open_dialog(owner,key,factory):
    """Keep one window per page and owner, preserving any unsaved edits."""
    windows=getattr(owner,"_dialogs",None)
    if windows is None:
        windows={};owner._dialogs=windows
    existing=windows.get(key)
    if existing is not None:
        try:
            if existing.winfo_exists():
                existing.deiconify();existing.lift();existing.focus_set()
                return existing
        except tk.TclError:pass
        windows.pop(key,None)
    dialog=factory()
    if dialog is None:return None
    windows[key]=dialog
    def released(event):
        if event.widget is dialog and windows.get(key) is dialog:
            windows.pop(key,None)
    dialog.bind("<Destroy>",released,add="+")
    dialog.lift();dialog.focus_set()
    return dialog


class ScrollFrame(ttk.Frame):
    def __init__(self,master,**kwargs):
        super().__init__(master,**kwargs)
        canvas=tk.Canvas(self,highlightthickness=0,bg="#f1f5f9",width=300)
        scrollbar=ttk.Scrollbar(self,orient="vertical",command=canvas.yview)
        self.content=ttk.Frame(canvas)
        window=canvas.create_window((0,0),window=self.content,anchor="nw")
        self.content.bind("<Configure>",lambda e:canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>",lambda e:canvas.itemconfigure(window,width=e.width))
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left",fill="both",expand=True); scrollbar.pack(side="right",fill="y")
        canvas.bind("<MouseWheel>",lambda e:canvas.yview_scroll(-int(e.delta/120),"units"))
        self.canvas=canvas


class VehicleEditor(tk.Toplevel):
    def __init__(self,app):
        super().__init__(app)
        self.app=app; self.data=asdict(app.vehicle)
        self.title("共用車輛設定 · SI 單位"); self.geometry("920x750")
        ttk.Label(self,text="FSAE 四輪四馬達 · 雙 A 臂。雙擊編輯；比例 0–1。馬達 CSV 按單顆額定扭矩輸入。",padding=12).pack(fill="x")
        frame=ttk.Frame(self); frame.pack(fill="both",expand=True,padx=12)
        self.tree=ttk.Treeview(frame,columns=("key","value"),show="tree headings")
        self.tree.heading("#0",text="參數 / 單位"); self.tree.heading("key",text="Python 欄位"); self.tree.heading("value",text="數值（雙擊）")
        self.tree.column("#0",width=345); self.tree.column("key",width=195); self.tree.column("value",width=300)
        scroll=ttk.Scrollbar(frame,command=self.tree.yview); self.tree.configure(yscrollcommand=scroll.set)
        self.tree.pack(side="left",fill="both",expand=True); scroll.pack(side="right",fill="y")
        for key,value in self.data.items(): self.tree.insert("","end",iid=key,text=LABELS.get(key,key),values=(key,format_value(value)))
        self.tree.bind("<Double-1>",self.edit)
        buttons=ttk.Frame(self,padding=12); buttons.pack(fill="x")
        ttk.Button(buttons,text="套用設定",command=self.apply).pack(side="right")
        ttk.Button(buttons,text="頻率 / 彈簧換算",command=lambda:self.app.open_rate_editor(self)).pack(side="left")
        ttk.Button(buttons,text="選擇馬達 CSV",command=self.select_motor_csv).pack(side="left",padx=8)
        ttk.Button(buttons,text="預覽馬達查表",command=self.preview_motor_curve).pack(side="left")

    def edit(self,event):
        key=self.tree.identify_row(event.y)
        if not key:return
        return open_dialog(self,"parameter",lambda:self.parameter_dialog(key))

    def parameter_dialog(self,key):
        old=self.data[key]
        dialog=tk.Toplevel(self); dialog.title(LABELS.get(key,key)); dialog.geometry("650x310")
        ttk.Label(dialog,text=LABELS.get(key,key),padding=10).pack(anchor="w")
        text=tk.Text(dialog,height=9,font=("Consolas",11),wrap="word")
        text.pack(fill="both",expand=True,padx=10)
        text.insert("1.0",json.dumps(old,indent=2) if isinstance(old,list) else str(old))
        dialog.parameter_key=key;dialog.text=text
        def commit():
            try:
                raw=text.get("1.0","end").strip()
                new=json.loads(raw) if isinstance(old,list) else raw if isinstance(old,str) else float(raw)
                candidate={**self.data,key:new}; Vehicle.from_dict(candidate)
                self.data[key]=new; self.tree.set(key,"value",format_value(new)); dialog.destroy()
            except (ValueError,TypeError) as exc:messagebox.showerror("輸入不正確",str(exc),parent=dialog)
        ttk.Button(dialog,text="確定",command=commit).pack(pady=10)
        text.focus_set()
        return dialog

    def select_motor_csv(self):
        path=filedialog.askopenfilename(parent=self,filetypes=[("CSV","*.csv")])
        if not path:return
        try:
            read_motor_curve(path)
            chosen=Path(path).resolve();root=Path(__file__).resolve().parent.parent
            try:stored=chosen.relative_to(root).as_posix()
            except ValueError:stored=str(chosen)
            self.data["motor_curve_csv"]=stored;self.tree.set("motor_curve_csv","value",stored)
        except (OSError,ValueError,KeyError,TypeError) as exc:messagebox.showerror("CSV 錯誤",str(exc),parent=self)

    def preview_motor_curve(self):
        return open_dialog(self,"motor_preview",self.motor_preview_dialog)

    def motor_preview_dialog(self):
        try:curve=read_motor_curve(self.data["motor_curve_csv"])
        except (OSError,ValueError) as exc:messagebox.showerror("CSV 錯誤",str(exc),parent=self);return
        dialog=tk.Toplevel(self);dialog.title("單顆馬達 · RPM → 扭矩查表");dialog.geometry("1000x650")
        ttk.Label(dialog,text=f"{curve.source}\n{len(curve.rpm)} 點 · 最高 {curve.max_rpm:g} RPM · 每顆峰值 {max(curve.torque):.4g} Nm\n四顆馬達共用本表；前後各自固定減速比。超過表格最高轉速不外插。",padding=12).pack(fill="x")
        chart=Chart(dialog);chart.pack(fill="both",expand=True)
        chart.set({"title":"Single-motor speed / torque map","xlabel":"Motor RPM","ylabel":"Torque N m",
            "series":[{"name":"CSV torque per motor","x":curve.rpm,"y":curve.torque}]})
        dialog.chart=chart
        return dialog

    def apply(self):
        try:
            v=Vehicle.from_dict(self.data)
            read_motor_curve(v.motor_curve_csv)
            self.app.vehicle=v; self.app.invalidate(); self.destroy()
        except (ValueError,TypeError,OSError) as exc:messagebox.showerror("設定錯誤",str(exc),parent=self)


class RateEditor(tk.Toplevel):
    def __init__(self,app,editor=None):
        super().__init__(editor if editor is not None else app); self.app=app; self.editor=editor
        self.title("懸吊頻率、彈簧與運動比"); self.geometry("560x510")
        ttk.Label(self,text="總運動比 = 懸吊比 × 搖臂比；kw = kcoil / MR²。\n選定一個目標，保留其餘設定。",padding=15).pack(fill="x")
        self.axle=tk.StringVar(value="front"); self.mode=tk.StringVar(value="frequency")
        for label,var,values in (("車軸",self.axle,["front","rear"]),("輸入目標",self.mode,["frequency","wheel_rate","coil_rate","motion_ratio","arb_tip_rate"])):
            ttk.Label(self,text=label).pack(); ttk.Combobox(self,textvariable=var,values=values,state="readonly").pack(pady=5)
        self.value=tk.StringVar(value="3.0"); ttk.Entry(self,textvariable=self.value).pack(pady=10)
        ttk.Label(self,text="frequency: Hz · rate: N/m · motion_ratio: 比值\narb_tip_rate: 防傾桿端剛性（經防傾桿運動比換算）").pack()
        self.info=ttk.Label(self,text="",padding=15); self.info.pack()
        ttk.Button(self,text="計算並套用",command=self.apply).pack(pady=10)
        ttk.Button(self,text="前後相同頻率與阻尼比（以前軸為準）",command=self.sync).pack(pady=10)
        self.refresh()

    def vehicle(self):return Vehicle.from_dict(self.editor.data) if self.editor else self.app.vehicle
    def refresh(self):
        v=self.vehicle()
        self.info.configure(text="\n".join(f'{a}: {physics.frequency(getattr(v,"wheel_rate_"+a),v.sprung(f)/2):.3f} Hz · coil {getattr(v,"wheel_rate_"+a)*v.ratio(f)**2:.0f} N/m'
            for f,a in ((True,"front"),(False,"rear"))))

    def save(self,v):
        v.validate()
        if self.editor:
            self.editor.data=asdict(v)
            for k,value in self.editor.data.items():self.editor.tree.set(k,"value",format_value(value))
        else:self.app.vehicle=v;self.app.invalidate()
        self.refresh()

    def apply(self):
        try:
            v=copy.deepcopy(self.vehicle()); a=self.axle.get(); front=a=="front"; value=float(self.value.get())
            if not math.isfinite(value) or value<=0:raise ValueError("輸入必須大於零")
            old=getattr(v,"wheel_rate_"+a); coil=old*v.ratio(front)**2; mode=self.mode.get()
            if mode=="frequency":rate=v.sprung(front)/2*(2*math.pi*value)**2
            elif mode=="wheel_rate":rate=value
            elif mode=="coil_rate":
                setattr(v,"motion_ratio_"+a,math.sqrt(value/old)/getattr(v,"bell_ratio_"+a));rate=old
            elif mode=="motion_ratio":
                setattr(v,"motion_ratio_"+a,value);rate=coil/v.ratio(front)**2
            else:
                width=getattr(v,"track_"+a);mr=getattr(v,"arb_motion_ratio_"+a)
                setattr(v,"arb_"+a,value/(mr*mr)*width*width/2);rate=old
            setattr(v,"wheel_rate_"+a,rate)
            if mode in ("frequency","wheel_rate"):setattr(v,"motion_ratio_"+a,math.sqrt(coil/rate)/getattr(v,"bell_ratio_"+a))
            self.save(v)
        except (ValueError,OverflowError) as exc:messagebox.showerror("換算錯誤",str(exc),parent=self)

    def sync(self):
        v=copy.deepcopy(self.vehicle()); mass_ratio=v.sprung(False)/v.sprung(True)
        v.wheel_rate_rear=v.wheel_rate_front*mass_ratio
        v.damping_comp_rear=v.damping_comp_front*mass_ratio; v.damping_reb_rear=v.damping_reb_front*mass_ratio
        self.save(v)


class GeometryEditor(tk.Toplevel):
    def __init__(self,app):
        super().__init__(app);self.app=app;self.geo=copy.deepcopy(app.geo)
        self.title("懸吊與轉向硬點 · mm");self.geometry("1010x780")
        ttk.Label(self,text="前後固定為雙 A 臂。x 向前，y 向左，z 向上；前軸 x=0、後軸 x=-軸距。\n所有硬點為 mm。每角上/下臂各三點，加上 ROD_IN / ROD_OUT。",padding=12).pack(fill="x")
        frame=ttk.Frame(self);frame.pack(fill="both",expand=True,padx=12)
        self.tree=ttk.Treeview(frame,columns=("x","y","z"),show="tree headings",selectmode="browse")
        self.tree.heading("#0",text="硬點 / 雙擊編輯");self.tree.column("#0",width=310)
        for col in ("x","y","z"):self.tree.heading(col,text=col+" mm");self.tree.column(col,width=180,anchor="e")
        scroll=ttk.Scrollbar(frame,command=self.tree.yview);self.tree.configure(yscrollcommand=scroll.set)
        self.tree.pack(side="left",fill="both",expand=True);scroll.pack(side="right",fill="y")
        self.tree.bind("<Double-1>",self.edit)
        buttons=ttk.Frame(self,padding=10);buttons.pack(fill="x")
        for text,command in (("套用",self.apply),("左側鏡射至右側",self.mirror),("重設硬點",self.reset),
                ("匯入 CSV",self.import_csv),("匯出 CSV",self.export_csv)):
            ttk.Button(buttons,text=text,command=command).pack(side="left",padx=4)
        self.populate()

    def populate(self):
        self.tree.delete(*self.tree.get_children())
        for group in ("corners",):
            for corner,points in self.geo[group].items():
                parent=self.tree.insert("","end",iid=group+"/"+corner,text=group+" / "+corner,open=True)
                for key,point in points.items():self.tree.insert(parent,"end",iid=group+"/"+corner+"/"+key,text=key,values=[f"{x*1000:.4f}" for x in point])

    def edit(self,event):
        row=self.tree.identify_row(event.y);parts=row.split("/")
        if len(parts)!=3:return
        a,b,c=parts;point=self.geo[a][b][c]
        raw=simpledialog.askstring("編輯硬點",f"{b}/{c}：x, y, z（mm）",initialvalue=", ".join(f"{x*1000:.6g}" for x in point),parent=self)
        if raw is None:return
        try:
            values=[float(s.strip())/1000 for s in raw.split(",")]
            if len(values)!=3 or not all(math.isfinite(x) and abs(x)<50 for x in values):raise ValueError("請輸入三個有效座標，以逗號分開")
            self.geo[a][b][c]=values;self.tree.item(row,values=[f"{x*1000:.4f}" for x in values])
        except ValueError as exc:messagebox.showerror("座標錯誤",str(exc),parent=self)

    def mirror(self):
        for left,right in (("FL","FR"),("RL","RR")):
            self.geo["corners"][right]={k:[p[0],-p[1],p[2]] for k,p in self.geo["corners"][left].items()}
        self.populate()

    def reset(self):self.geo=geom.default_geometry(self.app.vehicle);self.populate()

    def coordinate_rows(self):
        return [{"group":group,"corner":corner,"point":key,"x_mm":p[0]*1000,"y_mm":p[1]*1000,"z_mm":p[2]*1000}
            for group in ("corners",) for corner,points in self.geo[group].items() for key,p in points.items()]

    def export_csv(self):
        path=filedialog.asksaveasfilename(parent=self,defaultextension=".csv")
        if path:write_csv(path,self.coordinate_rows())

    def import_csv(self):
        path=filedialog.askopenfilename(parent=self,filetypes=[("CSV","*.csv")])
        if not path:return
        try:
            candidate=copy.deepcopy(self.geo)
            with open(path,encoding="utf-8-sig",newline="") as f:
                for r in csv.DictReader(f):candidate[r["group"]][r["corner"]][r["point"]]=[float(r[k])/1000 for k in ("x_mm","y_mm","z_mm")]
            geom.validate_geometry(candidate);self.geo=candidate;self.populate()
        except (ValueError,KeyError,OSError) as exc:messagebox.showerror("CSV 錯誤",str(exc),parent=self)

    def apply(self):
        try:
            geom.validate_geometry(self.geo);geom.solve_vehicle(self.app.vehicle,self.geo)
            self.app.geo=self.geo;self.app.invalidate();self.destroy()
        except (ValueError,KeyError) as exc:messagebox.showerror("幾何錯誤",str(exc),parent=self)


class TrackEditor(tk.Toplevel):
    def __init__(self,app):
        super().__init__(app);self.app=app;self.title("自訂賽道分段 · CSV");self.geometry("760x650")
        ttk.Label(self,text="length_m：分段長度 · radius_m：正=左彎、負=右彎、0=直線\ngrade_percent：上坡正（百分比）。封閉圈的速度邊界週期化；幾何輪廓不會自動閉合。",padding=12).pack()
        self.text=tk.Text(self,font=("Consolas",12));self.text.pack(fill="both",expand=True,padx=12)
        self.populate()
        bar=ttk.Frame(self,padding=10);bar.pack(fill="x")
        for label,command in (("套用",self.apply),("匯入分段 CSV",self.import_csv),("匯入中心線 x/y/z CSV",self.import_centerline),("匯出 CSV",self.export_csv)):
            ttk.Button(bar,text=label,command=command).pack(side="left",padx=3)

    def populate(self):
        self.text.delete("1.0","end");self.text.insert("1.0","length_m,radius_m,grade_percent\n")
        for r in self.app.track:self.text.insert("end",f'{r["length_m"]},{r["radius_m"]},{r.get("grade_percent",0)}\n')

    def parse(self):
        from .lap import validate_track
        rows=[{k:float(r.get(k) or 0) for k in ("length_m","radius_m","grade_percent")} for r in csv.DictReader(io.StringIO(self.text.get("1.0","end")))]
        validate_track(rows);return rows

    def apply(self):
        try:self.app.track=self.parse();self.app.track_name="自訂賽道";self.app.invalidate();self.destroy()
        except (ValueError,KeyError) as exc:messagebox.showerror("賽道錯誤",str(exc),parent=self)

    def import_csv(self):
        path=filedialog.askopenfilename(parent=self,filetypes=[("CSV","*.csv")])
        if path:
            try:
                track=read_track(path)
                self.text.delete("1.0","end");self.text.insert("1.0","length_m,radius_m,grade_percent\n")
                for r in track:self.text.insert("end",f'{r["length_m"]},{r["radius_m"]},{r["grade_percent"]}\n')
            except (OSError,ValueError,KeyError) as exc:messagebox.showerror("匯入錯誤",str(exc),parent=self)

    def import_centerline(self):
        path=filedialog.askopenfilename(parent=self,filetypes=[("CSV","*.csv")])
        if path:
            try:
                track=track_from_centerline(path,self.app.panels["lap"].vars["closed"].get())
                self.text.delete("1.0","end");self.text.insert("1.0","length_m,radius_m,grade_percent\n")
                for r in track:self.text.insert("end",f'{r["length_m"]},{r["radius_m"]},{r["grade_percent"]}\n')
            except (OSError,ValueError,KeyError) as exc:messagebox.showerror("匯入錯誤",str(exc),parent=self)

    def export_csv(self):
        path=filedialog.asksaveasfilename(parent=self,defaultextension=".csv")
        if path:
            try:write_csv(path,self.parse())
            except (ValueError,KeyError,OSError) as exc:messagebox.showerror("匯出錯誤",str(exc),parent=self)


class ToolPanel(ttk.Frame):
    def __init__(self,app,key):
        super().__init__(app.notebook);self.app=app;self.key=key;self.vars={};self.result=None;self.result_revision=-1
        self.columnconfigure(1,weight=1);self.rowconfigure(0,weight=1)
        inputs=ScrollFrame(self,width=320);inputs.grid(row=0,column=0,sticky="nsew",padx=(0,10));inputs.grid_propagate(False)
        self.inputs=inputs.content
        ttk.Label(self.inputs,text=NAMES[key],font=("Microsoft JhengHei UI",15,"bold"),padding=(12,12)).pack(anchor="w")
        for name,label,default,lo,hi in FIELDS[key]:self.field(name,label,default)
        if key in ("steering","roll","distribution","lap"):
            self.check("use_geometry","從幾何讀取側傾中心",True)
            self.field("rc_front_mm","手動前 RC mm",40);self.field("rc_rear_mm","手動後 RC mm",50)
        if key in ("roll","distribution","lap"):self.check("aero_on","啟用共用車身空力",key=="lap")
        if key=="roll":self.check("anti_on","使用幾何 roll axis",True)
        if key=="damper":
            self.check("overlay","疊加 B 後軸",True)
            ttk.Button(self.inputs,text="從階躍阻尼載入 A/B",command=self.load_dampers).pack(fill="x",padx=12,pady=5)
            ttk.Button(self.inputs,text="將高速係數寫回車輛",command=self.apply_dampers).pack(fill="x",padx=12,pady=5)
        if key=="step":
            ttk.Button(self.inputs,text="頻率 / 彈簧 / 前後同步",command=app.open_rate_editor).pack(fill="x",padx=12,pady=5)
            ttk.Button(self.inputs,text="回彈阻尼設為壓縮阻尼",command=self.match_damping).pack(fill="x",padx=12,pady=5)
            ttk.Button(self.inputs,text="傳送至阻尼曲線",command=self.send_damping).pack(fill="x",padx=12,pady=5)
        if key in ("geometry","steering","roll"):
            ttk.Button(self.inputs,text="編輯懸吊 / 轉向硬點",command=app.open_geometry_editor).pack(fill="x",padx=12,pady=6)
        if key=="geometry":
            ttk.Button(self.inputs,text="反求 RC / 幾何 anti 目標",command=self.geometry_goals).pack(fill="x",padx=12,pady=5)
        if key=="steering":
            ttk.Button(self.inputs,text="設定 scrub / trail 目標",command=self.kingpin_goals).pack(fill="x",padx=12,pady=5)
        if key=="distribution":
            ttk.Button(self.inputs,text="調整防傾桿至中性平衡",command=self.optimize_balance).pack(fill="x",padx=12,pady=5)
        if key=="aero":
            self.check("estimate","估算翼 Cl / Cd",True)
            ttk.Button(self.inputs,text="高度 / 溫度換算空氣密度",command=self.density_helper).pack(fill="x",padx=12,pady=5)
            ttk.Button(self.inputs,text="此翼下壓 → 前車身 ClA（取代）",command=lambda:self.apply_wing("front")).pack(fill="x",padx=12,pady=5)
            ttk.Button(self.inputs,text="此翼下壓 → 後車身 ClA（取代）",command=lambda:self.apply_wing("rear")).pack(fill="x",padx=12,pady=5)
        if key=="lap":
            self.check("closed","封閉圈（速度週期邊界）",True)
            ttk.Label(self.inputs,text="賽道",padding=(12,5)).pack(anchor="w")
            self.course=tk.StringVar(value=app.track_name)
            box=ttk.Combobox(self.inputs,textvariable=self.course,values=list(PRESETS),state="readonly",width=25)
            box.pack(fill="x",padx=12,pady=4);box.bind("<<ComboboxSelected>>",self.select_course)
            ttk.Button(self.inputs,text="分段 / 中心線 CSV",command=app.open_track_editor).pack(fill="x",padx=12,pady=5)
            ttk.Button(self.inputs,text="參數敏感度比較",command=lambda:self.calculate(True)).pack(fill="x",padx=12,pady=5)
        self.run_button=ttk.Button(self.inputs,text="計算  F5",command=self.calculate,style="Accent.TButton")
        self.run_button.pack(fill="x",padx=12,pady=14)
        ttk.Label(self.inputs,text="內部使用 SI 單位。各工具共用同一份車輛與硬點。\n修改輸入後請重新計算。",wraplength=270,padding=12,foreground="#64748b").pack(fill="x")
        result_frame=ttk.Frame(self);result_frame.grid(row=0,column=1,sticky="nsew")
        result_frame.rowconfigure(1,weight=1);result_frame.columnconfigure(0,weight=1)
        self.summary=tk.StringVar(value="就緒 · 選擇參數後計算")
        ttk.Label(result_frame,textvariable=self.summary,padding=10,font=("Microsoft JhengHei UI",10)).grid(row=0,column=0,sticky="ew")
        self.output=ttk.Notebook(result_frame);self.output.grid(row=1,column=0,sticky="nsew")
        graph=ttk.Frame(self.output);self.output.add(graph,text="圖表 / 幾何")
        controls=ttk.Frame(graph,padding=5);controls.pack(fill="x")
        self.plot_name=tk.StringVar();self.plot_select=ttk.Combobox(controls,textvariable=self.plot_name,state="readonly",width=43)
        self.plot_select.pack(side="left");self.plot_select.bind("<<ComboboxSelected>>",lambda e:self.show_plot())
        ttk.Button(controls,text="重設視野",command=lambda:self.chart.reset()).pack(side="left",padx=8)
        self.playing=False;self.animation_after=None
        self.play_button=ttk.Button(controls,text="播放",command=self.toggle_animation)
        if key in ("step","geometry","steering","roll","lap"):self.play_button.pack(side="left",padx=2)
        self.body_motion=tk.BooleanVar(value=False)
        if key in ("geometry","steering","roll"):
            features=ttk.Frame(graph,padding=(5,0,5,5));features.pack(fill="x")
            for name,label in (("projection_lines","Projection lines（投影線）"),("kingpin_lines","Kingpin lines（主銷軸線）")):
                self.vars[name]=tk.BooleanVar(value=True)
                check=ttk.Checkbutton(features,text=label,variable=self.vars[name],
                                      command=self.feature_lines_changed,state="disabled")
                check.pack(side="left",padx=(0,16));setattr(self,name+"_check",check)
        if key in ("geometry","roll"):
            motion=ttk.Frame(graph,padding=(5,0,5,5));motion.pack(fill="x")
            self.body_motion_check=ttk.Checkbutton(motion,text="車身隨動（輪胎貼地）",variable=self.body_motion,command=self.motion_mode_changed,state="disabled")
            self.body_motion_check.pack(side="left")
            self.animation_note=tk.StringVar(value="假設：剛性車身、平坦路面；準靜態姿態預覽")
            ttk.Label(motion,textvariable=self.animation_note,foreground="#64748b",wraplength=500).pack(side="left",padx=12)
        self.chart=Chart(graph);self.chart.pack(fill="both",expand=True)
        self.metrics=self.make_table("數值摘要",("metric","value"))
        self.table=self.make_table("結果資料",())
        note=tk.Text(self.output,wrap="word",font=("Microsoft JhengHei UI",12),padx=25,pady=25)
        note.insert("1.0",NOTES[key]+"\n\n欄位為空值（—）時，代表該狀態沒有有效解、已達輪胎飽和，或時間內尚未收斂。\n\n完整公式、座標定義與與原站差異請參閱專案 README.md。")
        note.configure(state="disabled");self.output.add(note,text="模型 / 假設")
        exports=ttk.Frame(result_frame,padding=(0,8));exports.grid(row=2,column=0,sticky="ew")
        for label,kind in (("資料 CSV","csv"),("曲線 CSV","curve"),("圖表 SVG","svg"),("圖表 PNG","png"),("報告 / PDF","html")):
            ttk.Button(exports,text=label,command=lambda k=kind:self.export(k)).pack(side="left",padx=3)
        if key in ("geometry","steering","roll"):
            for label,kind in (("STL 連桿","stl"),("STEP 線框","step")):
                ttk.Button(exports,text=label,command=lambda k=kind:self.export(k)).pack(side="left",padx=3)

    def field(self,name,label,default):
        frame=ttk.Frame(self.inputs,padding=(12,4));frame.pack(fill="x")
        ttk.Label(frame,text=label,wraplength=275).pack(anchor="w")
        var=tk.StringVar(value=str(default));self.vars[name]=var
        ttk.Entry(frame,textvariable=var).pack(fill="x",pady=(3,0))

    def check(self,name,label,default):
        var=tk.BooleanVar(value=default);self.vars[name]=var
        ttk.Checkbutton(self.inputs,text=label,variable=var).pack(anchor="w",padx=12,pady=5)

    def options(self):
        result=defaults(self.key)
        for key,var in self.vars.items():result[key]=bool(var.get()) if isinstance(var,tk.BooleanVar) else float(var.get())
        return result

    def make_table(self,title,columns):
        frame=ttk.Frame(self.output);self.output.add(frame,text=title)
        frame.rowconfigure(0,weight=1);frame.columnconfigure(0,weight=1)
        tree=ttk.Treeview(frame,columns=columns,show="headings")
        vertical=ttk.Scrollbar(frame,command=tree.yview);horizontal=ttk.Scrollbar(frame,orient="horizontal",command=tree.xview)
        tree.configure(yscrollcommand=vertical.set,xscrollcommand=horizontal.set)
        tree.grid(row=0,column=0,sticky="nsew");vertical.grid(row=0,column=1,sticky="ns");horizontal.grid(row=1,column=0,sticky="ew")
        for col in columns:tree.heading(col,text=col);tree.column(col,width=300)
        return tree

    def calculate(self,sensitivity=False):
        if self.app.busy:return
        try:
            options=self.options();v=copy.deepcopy(self.app.vehicle);geo=copy.deepcopy(self.app.geo);track=copy.deepcopy(self.app.track)
            v.validate()
            from .workflows import validate_options
            validate_options(self.key,options)
        except (ValueError,TypeError) as exc:messagebox.showerror("輸入錯誤",str(exc),parent=self.app);return
        self.app.busy=True;self.run_button.configure(state="disabled");self.summary.set("計算中…")
        revision=self.app.revision;snapshot_options=copy.deepcopy(options)
        snapshot_options["geometry_snapshot"]=geo
        if self.key=="lap":snapshot_options["track_snapshot"]=track
        def work():
            try:
                if sensitivity and self.key=="lap":
                    rf,rr=rc_values(v,geo,options)
                    vv=replace(v,mu_lat=v.mu_lat*options["grip_percent"]/100,mu_long=v.mu_long*options["grip_percent"]/100)
                    data=sensitivities(vv,track,rf,rr,step=options["step_m"],closed=options["closed"],
                        start_speed=options["start_kmh"]/3.6,aero_on=options["aero_on"],torque_scale=options["torque_percent"]/100)
                    for i,r in enumerate(data):r["scenario"]=i
                    out={"rows":data,"metrics":{"baseline_s":data[0]["lap_s"]},"note":NOTES[self.key],
                        "plots":[{"title":"Lap sensitivity (scenario index; see data table)","xlabel":"Scenario","ylabel":"Delta s", "series":[{"name":"Change vs baseline","x":[r["scenario"] for r in data],"y":[r["delta_s"] for r in data]}]}]}
                else:out=run_tool(self.key,v,geo,options,track)
                self.app.events.put((self,revision,out,None,v,snapshot_options))
            except Exception as exc:self.app.events.put((self,revision,None,exc,None,None))
        threading.Thread(target=work,daemon=True).start()

    def receive(self,revision,result,error,vehicle=None,options=None):
        self.run_button.configure(state="normal");self.app.busy=False
        if error:
            self.summary.set("計算失敗 · 請檢查輸入與模型適用範圍")
            messagebox.showerror("計算錯誤",str(error),parent=self.app);return
        self.result=result;self.result_revision=revision
        if vehicle is not None:self.result_vehicle=vehicle
        if options is not None:self.result_options=options
        self.plot_specs=list(result["plots"])
        if result.get("segments"):
            self.plot_specs=[{"title":"Geometry · "+view,"xlabel":"m","ylabel":"m","projection":view,"segments":result["segments"]} for view in ("Iso","Front","Rear","Side","Top")]+self.plot_specs
        self.plot_select.configure(values=[p["title"] for p in self.plot_specs]);self.plot_select.current(0);self.show_plot()
        self.metrics.delete(*self.metrics.get_children())
        for key,value in result["metrics"].items():self.metrics.insert("","end",values=(key,format_value(value)))
        columns=list(dict.fromkeys(k for row in result["rows"] for k in row))
        self.table.delete(*self.table.get_children());self.table.configure(columns=columns)
        for col in columns:self.table.heading(col,text=col);self.table.column(col,width=max(125,len(col)*8),stretch=False)
        for row in result["rows"][:3000]:self.table.insert("","end",values=[format_value(row.get(k)) for k in columns])
        shown=list(result["metrics"].items())[:3]
        self.summary.set("  ·  ".join(k+": "+format_value(value) for k,value in shown))
        self.app.status.set(f"{NAMES[self.key]}計算完成 · {len(result['rows'])} 列 · CSV 匯出完整資料")

    def show_plot(self):
        self.stop_animation()
        index=self.plot_select.current()
        if index>=0:
            spec=self.plot_specs[index]
            self.chart.set(spec)
            for name in ("projection_lines","kingpin_lines"):
                if hasattr(self,name+"_check"):
                    getattr(self,name+"_check").configure(state="normal" if "segments" in spec else "disabled")
            if hasattr(self,"body_motion_check"):
                self.body_motion_check.configure(state="normal" if "segments" in spec else "disabled")
                self.animation_note.set("假設：剛性車身、平坦路面；準靜態姿態預覽")
            if "segments" in spec:self.draw_geometry_frame()

    def stop_animation(self):
        self.playing=False
        self.play_button.configure(text="播放")
        if self.animation_after is not None:
            self.after_cancel(self.animation_after);self.animation_after=None

    def motion_mode_changed(self):
        if not self.result:return
        self.chart.view=None
        if self.playing:self.draw_geometry_frame(time.monotonic()-self.animation_start)
        else:self.show_plot()

    def feature_lines_changed(self):
        if not self.result or not self.chart.spec or "segments" not in self.chart.spec:return
        elapsed=time.monotonic()-self.animation_start if self.playing else None
        self.draw_geometry_frame(elapsed)

    def draw_geometry_frame(self,elapsed=None):
        """Always build from the result snapshot, never from a transformed frame."""
        spec=self.plot_specs[self.plot_select.current()]
        v=self.result_vehicle;o=self.result_options;geo=o.get("geometry_snapshot")
        phase=None if elapsed is None else math.sin(elapsed*1.5)
        try:
            if self.body_motion.get():
                pivot=[-v.wheelbase*(1-v.front_fraction),0.,0.]
                if self.key=="roll":
                    angle=self.result["metrics"]["Roll deg"]
                    if phase is not None:angle=phase*min(3,abs(angle) or 3)
                    rf=self.result["metrics"]["Front RC mm"]/1000
                    rr=self.result["metrics"]["Rear RC mm"]/1000
                    pivot[2]=physics.distribution(v,0.,rf,rr)["roll_axis"]
                    wheels=geom.solve_body_pose(v,geo,roll=math.radians(angle),pivot=pivot)
                else:
                    heave=o["heave_mm"]/1000
                    if phase is not None:heave+=phase*o["sweep_mm"]/1000*.7
                    wheels=geom.solve_body_pose(v,geo,heave,math.radians(o["roll_deg"]),math.radians(o["pitch_deg"]),pivot)
                segments=geom.body_scene(v,geo,wheels)
            elif phase is None:
                segments=spec["segments"]
                wheels=self.result["wheels"]
            else:
                if self.key=="steering":
                    wheels=geom.solve_vehicle(v,geo,rack=phase*o["sweep_rack_mm"]/1000,rear_ratio=o["rear_percent"]/100)
                elif self.key=="roll":
                    theta=math.radians(phase*min(3,abs(self.result["metrics"]["Roll deg"]) or 3))
                    travels={c:-(1 if c.endswith("L") else -1)*(v.track_front if c.startswith("F") else v.track_rear)/2*math.tan(theta) for c in CORNERS}
                    wheels=geom.solve_vehicle(v,geo,travels)
                else:wheels=geom.solve_vehicle(v,geo,{c:phase*o["sweep_mm"]/1000*.7 for c in CORNERS})
                segments=geom.segments(v,geo,wheels)
            overlays=geom.construction_segments(v,wheels,spec.get("projection","Iso"),
                self.vars["projection_lines"].get(),self.vars["kingpin_lines"].get())
            self.chart.spec={**spec,"segments":segments,"overlays":overlays};self.chart.draw()
            if hasattr(self,"animation_note"):
                self.animation_note.set("假設：剛性車身、平坦路面；準靜態姿態預覽")
            return True
        except ValueError as exc:
            self.stop_animation()
            if hasattr(self,"animation_note"):self.animation_note.set("姿態無法求解，請減少行程 / 角度："+str(exc))
            self.app.status.set("動畫停止：幾何姿態無法求解，請減少行程 / 角度。")
            return False

    def toggle_animation(self):
        if not self.result:return
        if self.playing:self.show_plot()
        else:
            self.playing=True;self.play_button.configure(text="停止")
            self.animation_start=time.monotonic();self.animate()

    def animate(self):
        self.animation_after=None
        if not self.playing:return
        if self.app.active()!=self:
            self.show_plot();return
        elapsed=time.monotonic()-self.animation_start
        spec=self.chart.spec
        if spec and "segments" in spec:
            if not self.draw_geometry_frame(elapsed):return
        elif spec and spec.get("series") and self.chart.view is not None:
            series=spec["series"][0];xs=[x for x in series["x"] if isinstance(x,(int,float))]
            if xs:
                x=min(xs)+(max(xs)-min(xs))*((elapsed/5)%1)
                l,t,r,b=self.chart.bounds;xmin,xmax,ymin,ymax=self.chart.view
                position=l+(x-xmin)/(xmax-xmin)*(r-l)
                self.chart.canvas.delete("playhead")
                self.chart.canvas.create_line(position,t,position,b,fill="#f8fafc",dash=(4,4),tags="playhead")
        self.animation_after=self.after(80,self.animate)

    def choose_axle(self):
        axle=simpledialog.askstring("選擇車軸","輸入 front 或 rear",initialvalue="front",parent=self.app)
        if axle is None:return None
        if axle.strip() not in ("front","rear"):raise ValueError("請輸入 front 或 rear")
        return axle.strip()

    def geometry_goals(self):
        try:
            axle=self.choose_axle()
            if axle is None:return
            current=geom.roll_center(self.app.vehicle,self.app.geo,axle)
            anti=geom.anti_percent(self.app.vehicle,self.app.geo,axle)
            target=simpledialog.askfloat("Roll center 目標","高度 mm",initialvalue=current[1]*1000 if current else 60,parent=self.app,minvalue=-300,maxvalue=800)
            if target is None:return
            goal=simpledialog.askfloat("幾何 anti 目標","Anti-dive / anti-squat %",initialvalue=anti if anti is not None else 20,parent=self.app,minvalue=-100,maxvalue=200)
            if goal is None:return
            self.app.geo=geom.tune_geometry(self.app.vehicle,self.app.geo,axle,target/1000,goal)
            self.app.invalidate();self.calculate()
        except ValueError as exc:messagebox.showerror("目標無法收斂",str(exc),parent=self.app)

    def kingpin_goals(self):
        try:
            axle=self.choose_axle()
            if axle is None:return
            scrub=simpledialog.askfloat("Scrub 目標","機械 scrub mm",initialvalue=30,parent=self.app,minvalue=-100,maxvalue=150)
            if scrub is None:return
            trail=simpledialog.askfloat("Trail 目標","機械 trail mm",initialvalue=35,parent=self.app,minvalue=-100,maxvalue=150)
            if trail is None:return
            self.app.geo=geom.set_kingpin_goals(self.app.vehicle,self.app.geo,axle,scrub/1000,trail/1000)
            self.app.invalidate();self.calculate()
        except ValueError as exc:messagebox.showerror("設定失敗",str(exc),parent=self.app)

    def optimize_balance(self):
        try:
            o=self.options();v=self.app.vehicle;rf,rr=rc_values(v,self.app.geo,o)
            ay=o["ay_g"]
            if abs(ay)<.01:raise ValueError("請先設定非零橫向 g")
            total=v.arb_front+v.arb_rear
            if total<=0:raise ValueError("至少需要非零的防傾桿總剛性")
            best=None
            for i in range(61):
                vv=replace(v,arb_front=total*i/60,arb_rear=total*(1-i/60))
                balance=physics.steady_balance(vv,ay,o["radius"],rf,rr,math.sqrt(abs(ay)*9.80665*o["radius"]),o.get("aero_on",False))
                value=balance["understeer_deg_g"]
                if value is not None and (best is None or abs(value)<best[0]):best=(abs(value),vv,value)
            if best is None:raise ValueError("所有候選均已飽和，請降低橫向 g")
            self.app.vehicle=best[1];self.app.invalidate();self.calculate()
            self.app.status.set(f"總防傾桿剛性保持不變；最佳可達不足轉向梯度 {best[2]:.4f} deg/g")
        except ValueError as exc:messagebox.showerror("平衡調整失敗",str(exc),parent=self.app)

    def load_dampers(self):
        v=self.app.vehicle
        for front,prefix in ((True,""),(False,"b_")):
            a="front" if front else "rear";ratio=v.ratio(front)**2
            for name,attr in (("c","damping_comp_"),("r","damping_reb_")):
                high=getattr(v,attr+a)*ratio
                for suffix,value in (("_high",high),("_low",5*high),("_knee",.05)):
                    self.vars[prefix+name+suffix].set(str(value))
        self.vars["end"].set("0.4");self.vars["overlay"].set(True)

    def apply_dampers(self):
        try:
            o=self.options();v=copy.deepcopy(self.app.vehicle)
            for front,prefix in ((True,""),(False,"b_")):
                a="front" if front else "rear"
                setattr(v,"damping_comp_"+a,o[prefix+"c_high"]/v.ratio(front)**2)
                setattr(v,"damping_reb_"+a,o[prefix+"r_high"]/v.ratio(front)**2)
            v.validate();self.app.vehicle=v;self.app.invalidate()
        except ValueError as exc:messagebox.showerror("錯誤",str(exc),parent=self.app)

    def match_damping(self):
        v=self.app.vehicle;v.damping_reb_front=v.damping_comp_front;v.damping_reb_rear=v.damping_comp_rear
        self.app.invalidate();self.calculate()

    def send_damping(self):
        panel=self.app.panels["damper"];panel.load_dampers();self.app.notebook.select(panel);panel.calculate()

    def density_helper(self):
        altitude=simpledialog.askfloat("空氣密度","海拔 m",initialvalue=0,parent=self.app)
        if altitude is None:return
        temp=simpledialog.askfloat("空氣密度","溫度 °C",initialvalue=15,parent=self.app)
        if temp is None:return
        try:self.app.vehicle.rho=physics.air_density(altitude,temp);self.app.invalidate();self.calculate()
        except ValueError as exc:messagebox.showerror("錯誤",str(exc),parent=self.app)

    def apply_wing(self,axle):
        if not self.result:messagebox.showinfo("先計算","先計算翼的 Cl 與下壓，再加入共用車身。",parent=self.app);return
        if self.result_revision!=self.app.revision or self.options()!= {k:v for k,v in self.result_options.items() if k!="geometry_snapshot"}:
            messagebox.showinfo("先重新計算","輸入已改變，請重新計算後再加入。",parent=self.app);return
        o=self.options();v=self.app.vehicle;cl=self.result["metrics"]["Cl"];area=o["chord"]*o["span"]
        if cl<0:
            messagebox.showerror("升力係數","共用車身 ClA 僅接受非負下壓；此翼目前產生升力。",parent=self.app);return
        setattr(v,"cla_"+axle,cl*area)
        self.app.invalidate();self.app.status.set("翼 ClA 已取代所選車軸車身 ClA；空力頁獨立翼仍存在，請避免重複計入。")

    def select_course(self,event=None):
        name=self.course.get();self.app.track=copy.deepcopy(PRESETS[name]);self.app.track_name=name
        self.vars["closed"].set("開放" not in name);self.app.invalidate()

    def export(self,kind):
        if not self.result:messagebox.showinfo("尚無結果","請先計算。",parent=self.app);return
        if self.playing:self.playing=False;self.show_plot()
        extension="csv" if kind=="curve" else kind
        path=filedialog.asksaveasfilename(parent=self.app,defaultextension="."+extension,
            initialfile=self.key+("_curve" if kind=="curve" else "")+"."+extension,filetypes=[(extension.upper(),"*."+extension)])
        if not path:return
        try:
            if kind=="csv":write_csv(path,self.result["rows"])
            elif kind=="curve":
                spec=self.chart.spec
                data=[{"curve":s["name"],"x":x,"y":y,"x_unit":spec.get("xlabel",""),"y_unit":spec.get("ylabel","")}
                    for s in spec.get("series",[]) for x,y in zip(s["x"],s["y"])]
                if not data:raise ValueError("請選擇數值曲線；硬點座標請在硬點編輯器匯出 CSV")
                write_csv(path,data)
            elif kind=="svg":Path(path).write_text(self.chart.svg(),encoding="utf-8")
            elif kind=="png":self.chart.png(path)
            elif kind=="html":write_report(path,NAMES[self.key],self.result_vehicle,self.result_options,self.result,self.chart.svg())
            elif kind=="stl":write_stl(path,self.result["segments"])
            elif kind=="step":write_step(path,self.result["segments"])
            self.app.status.set("已儲存 "+path+(" · 用瀏覽器開啟後列印為 PDF" if kind=="html" else ""))
        except (OSError,ValueError,KeyError) as exc:messagebox.showerror("匯出失敗",str(exc),parent=self.app)


class App(tk.Tk):
    def __init__(self):
        super().__init__();self.title("Vehicle Lab · FSAE 四輪馬達工具")
        self.geometry("1490x940");self.minsize(1160,740)
        self.vehicle=Vehicle();self.geo=geom.default_geometry(self.vehicle)
        self.track_name="示範 Autocross";self.track=copy.deepcopy(PRESETS[self.track_name])
        self.revision=0;self.busy=False;self.events=queue.Queue();self.project_path=None
        self._dialogs={};self._project_dialog_active=False
        style=ttk.Style(self);style.theme_use("clam")
        style.configure(".",font=("Microsoft JhengHei UI",10),background="#f1f5f9")
        style.configure("TNotebook.Tab",padding=(12,9));style.configure("TButton",padding=(10,7))
        style.configure("Accent.TButton",background="#2563eb",foreground="white",font=("Microsoft JhengHei UI",11,"bold"))
        style.map("Accent.TButton",background=[("active","#1d4ed8")])
        style.configure("Treeview",rowheight=28,fieldbackground="white",background="white")
        style.configure("TEntry",fieldbackground="white",padding=4)
        top=ttk.Frame(self,padding=(16,12));top.pack(fill="x")
        ttk.Label(top,text="VEHICLE LAB",font=("Segoe UI",20,"bold"),foreground="#0f172a").pack(side="left")
        ttk.Label(top,text="  FSAE · 雙 A 臂 · 4 馬達",foreground="#64748b").pack(side="left",padx=12)
        for label,command in (("使用說明",self.help),("開啟專案",self.load),("儲存專案",self.save),
                              ("雙 A 臂 / 硬點",self.open_geometry_editor),("共用車輛設定",self.open_vehicle_editor)):
            ttk.Button(top,text=label,command=command).pack(side="right",padx=4)
        self.profile=ttk.Label(self,padding=(18,0,18,10),foreground="#475569");self.profile.pack(fill="x")
        self.notebook=ttk.Notebook(self);self.notebook.pack(fill="both",expand=True,padx=12)
        self.panels={}
        for key,name in NAMES.items():
            panel=ToolPanel(self,key);self.panels[key]=panel;self.notebook.add(panel,text=name)
        self.status=tk.StringVar(value="所有計算在本機執行；不傳送車輛資料。")
        ttk.Label(self,textvariable=self.status,padding=(18,10),foreground="#475569").pack(fill="x")
        self.notebook.bind("<<NotebookTabChanged>>",self.tab_changed)
        self.bind("<F5>",lambda event:self.active().calculate())
        self.bind("<Control-s>",lambda event:self.save());self.bind("<Control-o>",lambda event:self.load())
        self.refresh_profile();self.after(100,self.poll);self.after(150,self.panels["step"].calculate)

    def active(self):return self.nametowidget(self.notebook.select())

    def open_vehicle_editor(self):return open_dialog(self,"vehicle",lambda:VehicleEditor(self))

    def open_geometry_editor(self):return open_dialog(self,"geometry",lambda:GeometryEditor(self))

    def open_rate_editor(self,editor=None):return open_dialog(self,"rate",lambda:RateEditor(self,editor))

    def open_track_editor(self):return open_dialog(self,"track",lambda:TrackEditor(self))

    def destroy(self):
        # Cancel queued polls/animations before destroying their Tcl commands.
        # This also permits repeated Tk sessions in integration tests.
        try:
            for timer in self.tk.splitlist(self.tk.call("after","info")):
                self.tk.call("after","cancel",timer)
        except tk.TclError:pass
        super().destroy()

    def refresh_profile(self):
        v=self.vehicle
        self.profile.configure(text=f"{v.name}   |   {v.mass:.0f} kg   |   軸距 {v.wheelbase:.3f} m   |   前重 {v.front_fraction*100:.1f}%   |   4×馬達 · 減速比 F {v.reduction_front:g} / R {v.reduction_rear:g}   |   雙 A 臂")

    def invalidate(self):
        self.revision+=1;self.refresh_profile()
        self.status.set("共用設定已更新；切換工具或按 F5 重新計算。")
        if "lap" in self.panels:self.panels["lap"].course.set(self.track_name)

    def tab_changed(self,event=None):
        panel=self.active()
        if not self.busy and panel.result_revision!=self.revision:self.after(30,panel.calculate)

    def poll(self):
        try:
            while True:
                panel,revision,result,error,vehicle,options=self.events.get_nowait()
                panel.receive(revision,result,error,vehicle,options)
        except queue.Empty:pass
        self.after(80,self.poll)

    def project_file_dialog(self,saving):
        # Native file dialogs run their own event loop, so shortcut callbacks
        # can re-enter while one is open. Opening and saving share this guard.
        if self._project_dialog_active:return None
        self._project_dialog_active=True
        try:
            options={"parent":self,"filetypes":[("Vehicle Lab project","*.json")]}
            if saving:return filedialog.asksaveasfilename(**options,defaultextension=".json",initialfile="vehicle_project.json")
            return filedialog.askopenfilename(**options)
        finally:self._project_dialog_active=False

    def save(self):
        path=self.project_file_dialog(True)
        if not path:return
        try:
            settings={key:panel.options() for key,panel in self.panels.items()}
            settings.update(track=self.track,track_name=self.track_name)
            write_project(path,self.vehicle,self.geo,settings);self.project_path=path;self.status.set("已儲存專案 "+path)
        except (ValueError,OSError) as exc:messagebox.showerror("儲存失敗",str(exc),parent=self)

    def load(self):
        path=self.project_file_dialog(False)
        if not path:return
        try:
            vehicle,geo,settings=read_project(path);geom.validate_geometry(geo);geom.solve_vehicle(vehicle,geo)
            read_motor_curve(vehicle.motor_curve_csv)
            from .lap import validate_track
            track=settings.get("track",self.track);validate_track(track)
            from .workflows import validate_options
            for key in NAMES:validate_options(key,{**defaults(key),**settings.get(key,{})})
            self.vehicle=vehicle;self.geo=geo;self.track=track;self.track_name=settings.get("track_name","自訂賽道")
            for key,panel in self.panels.items():
                for name,value in settings.get(key,{}).items():
                    if name in panel.vars:panel.vars[name].set(value)
            self.project_path=path;self.invalidate();self.active().calculate();self.status.set("已載入 "+path)
        except (ValueError,KeyError,TypeError,OSError) as exc:messagebox.showerror("載入失敗",str(exc),parent=self)

    def help(self):
        return open_dialog(self,"help",self.help_dialog)

    def help_dialog(self):
        dialog=tk.Toplevel(self);dialog.title("Vehicle Lab 使用說明");dialog.geometry("850x690")
        text=tk.Text(dialog,wrap="word",font=("Microsoft JhengHei UI",11),padx=20,pady=20);text.pack(fill="both",expand=True)
        readme=Path(__file__).resolve().parent.parent/"README.md"
        text.insert("1.0",readme.read_text(encoding="utf-8") if readme.exists() else "See README.md")
        text.configure(state="disabled")
        return dialog


def main():
    app=App();app.mainloop()


if __name__=="__main__":main()
