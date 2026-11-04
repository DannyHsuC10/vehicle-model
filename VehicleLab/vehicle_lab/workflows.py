"""Nine desktop workflows, separately callable for scripting and validation."""
import math
from .models import CORNERS, G
from . import physics as p
from . import geometry as geom
from .lap import simulate_lap, PRESETS
from .linalg import add, sub, rotate

# (key, Traditional Chinese label with displayed units, default, min, max)
FIELDS={
    "step": [("step_mm","路面階躍 mm",25,-200,200),("duration","模擬時間 s",2,.1,30),
             ("speed_kmh","車速 km/h",60,.1,500)],
    "damper": [("c_low","A 壓縮低速 N s/m",6480,0,1e6),("c_high","A 壓縮高速 N s/m",1296,0,1e6),
        ("c_knee","A 壓縮拐點 m/s",.05,0,5),("r_low","A 回彈低速 N s/m",8640,0,1e6),
        ("r_high","A 回彈高速 N s/m",1728,0,1e6),("r_knee","A 回彈拐點 m/s",.05,0,5),
        ("b_c_low","B 壓縮低速 N s/m",7200,0,1e6),("b_c_high","B 壓縮高速 N s/m",1440,0,1e6),
        ("b_c_knee","B 壓縮拐點 m/s",.05,0,5),("b_r_low","B 回彈低速 N s/m",9360,0,1e6),
        ("b_r_high","B 回彈高速 N s/m",1872,0,1e6),("b_r_knee","B 回彈拐點 m/s",.05,0,5),
        ("end","最高桿速 m/s",.3,.001,5)],
    "geometry": [("heave_mm","共同跳動 mm（正=壓縮）",0,-100,100),
        ("roll_deg","車身側傾 deg（正=右側低）",0,-8,8),("pitch_deg","車頭下俯 deg",0,-5,5),
        ("sweep_mm","行程掃描 ±mm",40,5,100)],
    "steering": [("rack_mm","齒條位移 mm（正=向左）",0,-80,80),("rear_percent","後轉向 / 前齒條 %",0,-100,100),
        ("sweep_rack_mm","齒條掃描 ±mm",20,1,80),("ax_g","縱向 g（正=煞車）",0,-3,3),
        ("ay_g","橫向 g（正=左轉）",0,-3,3)],
    "roll": [("ay_g","橫向 g（正=左轉）",1,-3,3),("radius","彎道半徑 m",20,2,5000)],
    "distribution": [("ay_g","橫向 g（正=左轉）",1,-3,3),("radius","彎道半徑 m",20,2,5000)],
    "aero": [("speed_kmh","車速 km/h",80,0,500),("max_speed_kmh","掃描最高 km/h",140,1,600),
        ("chord","翼弦 m",.22,.02,3),("span","翼展 m",1.1,.05,5),("angle","攻角 deg",8,-20,25),
        ("camber","翼彎度 %",2,0,9),("camber_at","最大彎度位置 %",40,10,90),
        ("thickness","厚度比 %",12,4,25),("position","翼位置 / 軸距 %（前軸=0）",108,-50,200),
        ("cl","手動 Cl（下壓為正）",.8,-2,4),("wing_cd","手動翼 Cd",.06,0,2)],
    "rigid": [("ax_g","縱向 g（正=煞車）",1,-3,3),("ay_g","橫向 g（正=左轉）",1,-3,3)],
    "lap": [("step_m","空間步長 m",2,1,50),("start_kmh","開放賽道起速 km/h",0,0,500),
        ("torque_percent","馬達扭矩倍率 %",100,1,500),("grip_percent","抓地倍率 %",100,10,300)]}

NAMES={"step":"階躍響應","damper":"阻尼曲線","geometry":"懸吊幾何","steering":"轉向",
       "roll":"側傾","distribution":"載重轉移分配","aero":"空氣力學","rigid":"剛體載重轉移","lap":"圈速"}

NOTES={
    "step":"前後各為單自由度簧上質量；沒有輪胎彈性、簧下運動或俯仰耦合。後軸時間平移 L/v。採物理階躍與 RK4；不仿照原站特殊的輸出轉換，數值不保證相同。阻尼為輪端值。",
    "damper":"連續雙線性阻尼；壓縮正、回彈負，橫軸為正桿速。係數為減震器桿端；從階躍匯入時乘總運動比平方。忽略遲滯、氣壓、摩擦和溫度。",
    "geometry":"前後均固定雙 A 臂，四角各由定長上/下臂、轉向桿與接地點行程求三維位姿。x 向前、y 向左、z 向上，前軸 x=0。無襯套柔度、碰撞或強度計算。解不到的行程保留缺口，請查看 status。",
    "steering":"沿用懸吊硬點與定長轉向拉桿。機械 trail、scrub 取轉向軸與地面交點；氣動 trail 以接地長度/6 估算，非輪胎實測。拉桿力不含方向機減速比、摩擦與助力。",
    "roll":"輪端彈簧與防傾桿剛性相加；幾何經 roll axis 作用於簧上側傾力矩，不重複折減剛性。曲線為準靜態剛性模型，抬輪後數值僅作診斷。",
    "distribution":"三條力矩路徑：簧下、幾何、彈性。ΔF 是由內輪移至外輪的力，外內輪差為 2ΔF。穩態平衡用簡化 Magic Formula 上升支求滑角；飽和會顯示空值。",
    "aero":"阻力與下壓隨速度平方變化；翼估算使用薄翼與有限翼展修正，無失速/地面效應/CFD。翼為獨立項，需按按鈕才加入共用車身 ClA。超出線性攻角範圍請改用手動係數。",
    "rigid":"縱向 m ax h/L；橫向以當下軸荷分配。沒有側傾中心、彈簧、防傾桿。負輪荷表示抬輪/模型失效，保留負值以維持力矩檢查；接地面積歸零。",
    "lap":"四輪各一顆獨立馬達：輪速→前/後固定減速比→CSV RPM查表→馬達扭矩→減速比與效率→輪扭矩。四輪分別受馬達與摩擦橢圓限制，無變速箱/差速器。CSV每次模擬只讀一次；超出表中最高RPM扭矩為零。煞車採液壓分配，未計再生制動、電池功率限制或電控向量分配。示範賽道非實測地圖。"}


def defaults(key):
    result={f[0]:f[2] for f in FIELDS[key]}
    result.update(use_geometry=True,rc_front_mm=40.,rc_rear_mm=50.,aero_on=False,
                  estimate=True,overlay=True,closed=True,anti_on=True)
    if key=="lap": result["aero_on"]=True
    if key in ("geometry","steering","roll"):
        result.update(projection_lines=True,kingpin_lines=True)
    return result


def validate_options(key,options):
    unknown=set(options)-set(defaults(key))
    if unknown:raise ValueError("Unsupported tool inputs: "+", ".join(sorted(unknown)))
    for name,label,default,lo,hi in FIELDS[key]:
        value=options[name]
        if not math.isfinite(value) or not lo<=value<=hi: raise ValueError(f"{label}: {lo} .. {hi}")
    for name in ("rc_front_mm","rc_rear_mm"):
        value=options.get(name,0)
        if not math.isfinite(value) or not -1000<=value<=2000: raise ValueError("Roll center must be -1000..2000 mm")


def rc_values(v,geo,options):
    if not options.get("use_geometry",True):
        return options["rc_front_mm"]/1000,options["rc_rear_mm"]/1000
    wheels=geom.solve_vehicle(v,geo)
    rc=[geom.roll_center(v,geo,a,wheels) for a in ("front","rear")]
    if any(r is None for r in rc): raise ValueError("Geometry has no finite roll center. Check mounts or use manual RC.")
    return rc[0][1],rc[1][1]


def plot(rows,x,ys,title,xlabel="",ylabel=""):
    series=[]
    for field,name in ys:
        series.append({"name":name,"x":[r.get(x) for r in rows],"y":[r.get(field) for r in rows]})
    return {"title":title,"xlabel":xlabel or x,"ylabel":ylabel,"series":series}


def run_tool(key,v,geo,options,track=None):
    v.validate(); geom.validate_geometry(geo); validate_options(key,options)
    o=options; metrics={}; plots=[]; rows=[]; extra={}
    if key=="step":
        times=[]
        for front,label in ((True,"Front"),(False,"Rear")):
            axle=v.axle(front); delay=0 if front else v.wheelbase/(o["speed_kmh"]/3.6)
            rr,mm=p.step_response(v.sprung(front)/2,getattr(v,"wheel_rate_"+axle),
                getattr(v,"damping_comp_"+axle),getattr(v,"damping_reb_"+axle),o["step_mm"]/1000,o["duration"],delay)
            for k,value in mm.items(): metrics[label+" "+k]=value
            for r in rr: r["axle"]=label; r["body_mm"]=r["body_m"]*1000
            rows.extend(rr)
            times.append((label,rr))
        metrics["Rear delay s"]=v.wheelbase/(o["speed_kmh"]/3.6)
        for y,title,unit in (("body_mm","Body response","mm"),("velocity_m_s","Body velocity","m/s")):
            plots.append({"title":title,"xlabel":"Time s","ylabel":unit,"series":[
                {"name":label,"x":[r["time_s"] for r in rr],"y":[r[y] for r in rr]} for label,rr in times]})
        tf=p.transmissibility(v.wheel_rate_front,v.sprung(True)/2,v.damping_comp_front,v.damping_reb_front)
        tr=p.transmissibility(v.wheel_rate_rear,v.sprung(False)/2,v.damping_comp_rear,v.damping_reb_rear)
        plots.append({"title":"Transmissibility","xlabel":"Frequency Hz","ylabel":"Body / road","series":[
            {"name":"Front","x":[r["frequency_Hz"] for r in tf],"y":[r["body_road_ratio"] for r in tf]},
            {"name":"Rear","x":[r["frequency_Hz"] for r in tr],"y":[r["body_road_ratio"] for r in tr]}]})
    elif key=="damper":
        for prefix,label in (("","A front"),("b_","B rear")):
            if prefix and not o.get("overlay",True): continue
            rr=p.damper_curves(*(o[prefix+k] for k in ("c_low","c_high","c_knee","r_low","r_high","r_knee")),o["end"])
            for r in rr: r["curve"]=label
            rows.extend(rr)
            for name in ("compression_N","rebound_N"):
                extra.setdefault("series",[]).append({"name":label+" "+name,"x":[r["shaft_m_s"] for r in rr],"y":[r[name] for r in rr]})
        plots=[{"title":"Damper dyno","xlabel":"Shaft speed m/s","ylabel":"Force N","series":extra["series"]}]
        metrics={"A compression HS at wheel N s/m":o["c_high"]/v.ratio(True)**2,
                 "A rebound HS at wheel N s/m":o["r_high"]/v.ratio(True)**2,
                 "B compression HS at wheel N s/m":o["b_c_high"]/v.ratio(False)**2}
    elif key=="geometry":
        travels={}
        for c in CORNERS:
            front=c.startswith("F"); side=1 if c.endswith("L") else -1
            track_width=v.track_front if front else v.track_rear
            pitch=(1-v.front_fraction if front else -v.front_fraction)*v.wheelbase*math.tan(math.radians(o["pitch_deg"]))
            travels[c]=o["heave_mm"]/1000-side*track_width/2*math.tan(math.radians(o["roll_deg"]))+pitch
        wheels=geom.solve_vehicle(v,geo,travels)
        extra["wheels"]=wheels
        extra["segments"]=geom.segments(v,geo,wheels)
        for axle in ("front","rear"):
            rc=geom.roll_center(v,geo,axle,wheels)
            metrics[axle+" RC height mm"]=None if rc is None else rc[1]*1000
            metrics[axle+" RC lateral mm"]=None if rc is None else rc[0]*1000
            metrics[axle+" geometric anti %"]=geom.anti_percent(v,geo,axle,wheels)
        rows=geom.geometry_sweep(v,geo,o["sweep_mm"]/1000)
        for name,title in (("camber_deg","Camber vs heave"),("caster_deg","Caster vs heave"),("toe_deg","Toe vs heave")):
            plots.append(plot(rows,"travel_mm",[(c+"_"+name,c) for c in CORNERS],title,"Contact travel mm","deg"))
        plots.append(plot(rows,"travel_mm",[("front_RC_mm","Front"),("rear_RC_mm","Rear")],"Roll center vs heave","Contact travel mm","Height mm"))
        metrics["Unreachable sweep samples"]=sum(r["status"]!="ok" for r in rows)
        extra["corner_rows"]=[{"corner":c,**{k:w[k] for k in ("camber_deg","caster_deg","toe_deg","residual_m")}} for c,w in wheels.items()]
    elif key=="steering":
        rf,rr=rc_values(v,geo,o)
        rows,wheels,ack=geom.steering_metrics(v,geo,o["rack_mm"]/1000,o["rear_percent"]/100,o["ax_g"],o["ay_g"],rf,rr)
        extra["wheels"]=wheels
        metrics={"Front Ackermann % at this rack":ack,
            "Rear Ackermann % at this rack":geom.ackermann_percent(rows[2]["steer_deg"],rows[3]["steer_deg"],v.wheelbase,v.track_rear)}
        extra["segments"]=geom.segments(v,geo,wheels)
        sweep=[]
        for rack in p.linspace(-o["sweep_rack_mm"],o["sweep_rack_mm"],25):
            item={"rack_mm":rack}
            try:
                ws=geom.solve_vehicle(v,geo,rack=rack/1000,rear_ratio=o["rear_percent"]/100)
                for c,w in ws.items(): item[c]=w["steer_deg"]; item[c+"_camber"]=w["camber_deg"]
            except ValueError: pass
            sweep.append(item)
        plots.append(plot(sweep,"rack_mm",[(c,c) for c in CORNERS],"Road steer vs rack","Rack mm","Road steer deg"))
        plots.append(plot(sweep,"rack_mm",[(c+"_camber",c) for c in CORNERS],"Camber vs rack","Rack mm","Camber deg"))
        bump=geom.geometry_sweep(v,geo,count=25)
        plots.append(plot(bump,"travel_mm",[(c+"_toe_deg",c) for c in CORNERS],"Bump steer","Travel mm","Toe-in deg"))
        ackrows=[]
        for item in sweep:
            if item.get("FL") is None or item.get("FR") is None: continue
            left,right=item["FL"],item["FR"]
            inside,outside=(abs(left),abs(right)) if left+right>0 else (abs(right),abs(left))
            ideal=math.degrees(math.atan(1/(1/max(math.tan(math.radians(outside)),1e-12)-v.track_front/v.wheelbase))) if outside>.1 else 0
            ackrows.append({"outside":outside,"inside":inside,"ideal":ideal})
        plots.append(plot(ackrows,"outside",[("inside","Actual"),("ideal","Ideal Ackermann")],"Inside vs outside","Outside deg","Inside deg"))
        extra["sweep_rows"]=sweep
    elif key in ("roll","distribution"):
        rf,rr=rc_values(v,geo,o)
        if key=="roll" and not o.get("anti_on",True): rf=rr=0.
        speed=math.sqrt(abs(o["ay_g"])*G*o["radius"])
        d=p.steady_balance(v,o["ay_g"],o["radius"],rf,rr,speed,o.get("aero_on",False))
        stiff=p.roll_stiffness(v)
        limit,end=p.lift_limit(v,rf,rr)
        metrics={"Front RC mm":rf*1000,"Rear RC mm":rr*1000,"Roll deg":d["roll_deg"],
            "Roll gradient deg/g":d["gradient_deg_g"],"Speed km/h":speed*3.6,
            "Front stiffness %":100*d["stiffness_front_fraction"],
            "Front moment share %":None if d["front_moment_fraction"] is None else d["front_moment_fraction"]*100,
            "Moment balance error N m":d["moment_error"],"No-aero lift g":limit,"First lift axle":end,
            "Total roll N m/deg":stiff["total"]*math.pi/180,
            "Front grip remaining %":d["grip_percent"][0],"Rear grip remaining %":d["grip_percent"][1],
            "Road wheel steer deg":d["steer_deg"],"Understeer deg/g":d["understeer_deg_g"]}
        rows=[{"corner":c,"static_N":d["static"][i],"load_N":d["loads"][i],"slip_deg":d["slip_deg"][i],
               "lateral_force_N":d["forces_N"][i]} for i,c in enumerate(CORNERS)]
        paths=[{"path":name,"front_Nm":values[0],"rear_Nm":values[1]} for name,values in d["paths"].items()]
        extra["path_rows"]=paths
        for r in paths: metrics[r["path"]+" front / rear N m"]=f'{r["front_Nm"]:.2f} / {r["rear_Nm"]:.2f}'
        scan=[]
        for ay in p.linspace(-min(limit,2.5),min(limit,2.5),41):
            dd=p.distribution(v,ay,rf,rr,math.sqrt(abs(ay)*G*o["radius"]),o.get("aero_on",False))
            item={"ay_g":ay,"roll_deg":dd["roll_deg"]}
            for i,c in enumerate(CORNERS): item[c]=dd["loads"][i]
            scan.append(item)
        plots.append(plot(scan,"ay_g",[(c,c) for c in CORNERS],"Corner loads","Lateral g","N"))
        plots.append(plot(scan,"ay_g",[("roll_deg","Body roll")],"Roll vs lateral acceleration","Lateral g","deg"))
        if key=="roll":
            theta=math.radians(d["roll_deg"])
            travel={c:-(1 if c.endswith("L") else -1)*(v.track_front if c.startswith("F") else v.track_rear)/2*math.tan(theta)+d["roll_axis"]*(1-1/math.cos(theta)) for c in CORNERS}
            pivot=[0,0,d["roll_axis"]]
            world=lambda point:add(pivot,rotate(sub(point,pivot),[theta,0,0]))
            try:
                wheels=geom.solve_vehicle(v,geo,travel)
                extra["segments"]=[(world(a),world(b),kind) for a,b,kind in geom.segments(v,geo,wheels)]
                extra["wheels"]={c:{**w,"points":{k:world(point) for k,point in w["points"].items()},
                    **{k:world(w[k]) for k in ("center","contact","axis_origin")},
                    **{k:rotate(w[k],[theta,0,0]) for k in ("forward","up","axis")}}
                    for c,w in wheels.items()}
                for r in rows:
                    w=wheels[r["corner"]]; r["travel_mm"]=travel[r["corner"]]*1000
                    r["camber_ground_deg"]=w["camber_deg"]-(1 if r["corner"].endswith("L") else -1)*d["roll_deg"]
                    r["toe_deg"]=w["toe_deg"]
            except ValueError as exc: metrics["Kinematics status"]=str(exc)
            scan_geometry=[]
            for ay in p.linspace(-min(.95*limit,2.),min(.95*limit,2.),21):
                dd=p.distribution(v,ay,rf,rr);th=math.radians(dd["roll_deg"])
                item={"roll_deg":dd["roll_deg"]}
                ts={c:-(1 if c.endswith("L") else -1)*(v.track_front if c.startswith("F") else v.track_rear)/2*math.tan(th) for c in CORNERS}
                try:
                    ws=geom.solve_vehicle(v,geo,ts)
                    for c,w in ws.items():
                        item[c+"_camber"]=w["camber_deg"]-(1 if c.endswith("L") else -1)*dd["roll_deg"]
                        item[c+"_toe"]=w["toe_deg"]
                    for axle in ("front","rear"):
                        rc=geom.roll_center(v,geo,axle,ws)
                        if rc is not None:
                            point=add([0,0,dd["roll_axis"]],rotate([0,rc[0],rc[1]-dd["roll_axis"]],[th,0,0]))
                            item[axle+"_rc_z"]=point[2]*1000;item[axle+"_rc_y"]=point[1]*1000
                except ValueError:pass
                scan_geometry.append(item)
            plots.append(plot(scan_geometry,"roll_deg",[(c+"_camber",c) for c in CORNERS],"Camber vs roll","Body roll deg","Ground camber deg"))
            plots.append(plot(scan_geometry,"roll_deg",[(c+"_toe",c) for c in CORNERS],"Toe vs roll","Body roll deg","Toe-in deg"))
            plots.append(plot(scan_geometry,"roll_deg",[("front_rc_z","Front"),("rear_rc_z","Rear")],"Roll center height vs roll","Body roll deg","Height mm"))
            plots.append(plot(scan_geometry,"roll_deg",[("front_rc_y","Front"),("rear_rc_y","Rear")],"Roll center lateral vs roll","Body roll deg","Left + mm"))
        else:
            balance=[]
            for ay in p.linspace(.05,min(1.5,limit),31):
                bb=p.steady_balance(v,ay,o["radius"],rf,rr,math.sqrt(ay*G*o["radius"]),o.get("aero_on",False))
                balance.append({"ay_g":ay,"steer":bb["steer_deg"],"understeer":bb["understeer_deg_g"]})
            plots.append(plot(balance,"ay_g",[("steer","Road steer")],"Steady corner balance","Lateral g","deg"))
    elif key=="aero":
        kw={"chord":o["chord"],"span":o["span"],"angle":o["angle"],"camber":o["camber"]/100,
            "camber_at":o["camber_at"]/100,"thickness":o["thickness"]/100,"position":o["position"]/100,
            "estimate":o.get("estimate",True),"cl":o["cl"],"wing_cd":o["wing_cd"]}
        metrics=p.aero(v,o["speed_kmh"]/3.6,**kw)
        rows=[p.aero(v,s/3.6,**kw) for s in p.linspace(0,o["max_speed_kmh"],121)]
        plots.append(plot(rows,"speed_kmh",[("body_power_kW","Body"),("total_power_kW","Body + wing")],"Aero power","km/h","kW"))
        plots.append(plot(rows,"speed_kmh",[("front_down_N","Front"),("rear_down_N","Rear"),("wing_down_N","Wing")],"Downforce","km/h","N"))
        plots.append(plot(rows,"speed_kmh",[("front_ride_mm","Front"),("rear_ride_mm","Rear")],"Ride-height change","km/h","mm"))
        profile=[]
        for x in p.linspace(0,1,101):
            m=o["camber"]/100; pp=o["camber_at"]/100
            yc=m/pp**2*(2*pp*x-x*x) if x<pp else m/(1-pp)**2*(1-2*pp+2*pp*x-x*x)
            yt=5*o["thickness"]/100*(.2969*math.sqrt(x)-.126*x-.3516*x*x+.2843*x**3-.1036*x**4)
            profile.append({"x":x*o["chord"],"upper":(yc+yt)*o["chord"],"lower":(yc-yt)*o["chord"]})
        plots.append(plot(profile,"x",[("upper","Upper"),("lower","Lower")],"Wing section (unrotated)","Chord m","m"))
    elif key=="rigid":
        d=p.rigid_transfer(v,o["ax_g"],o["ay_g"])
        for i,c in enumerate(CORNERS):
            front=i<2
            patch=p.contact_patch(d["loads"][i],v.pressure_front if front else v.pressure_rear,
                v.tire_width_front if front else v.tire_width_rear,(v.tire_diameter_front if front else v.tire_diameter_rear)/2)
            rows.append({"corner":c,"static_N":d["static"][i],"load_N":d["loads"][i],"load_kg":d["loads"][i]/G,
                "patch_area_cm2":patch["area_m2"]*10000,"patch_length_mm":patch["length_m"]*1000,
                "patch_width_mm":patch["width_m"]*1000,"lifted":patch["lifted"]})
        metrics={"Front axle N":d["axles"][0],"Rear axle N":d["axles"][1],
            "Longitudinal transfer N":d["longitudinal_delta_N"],"Total vertical N":sum(d["loads"]),
            "Vertical balance error N":sum(d["loads"])-v.mass*G}
        scan=[]
        for ay in p.linspace(-2,2,81):
            dd=p.rigid_transfer(v,o["ax_g"],ay)
            scan.append({"ay_g":ay,**dict(zip(CORNERS,dd["loads"]))})
        plots.append(plot(scan,"ay_g",[(c,c) for c in CORNERS],"Rigid corner loads","Lateral g","N"))
        patches=[]
        for i,c in enumerate(CORNERS):
            front=i<2; side=1 if c.endswith("L") else -1
            center=[0 if front else -v.wheelbase,side*(v.track_front if front else v.track_rear)/2]
            length=rows[i]["patch_length_mm"]/1000; width=rows[i]["patch_width_mm"]/1000
            pts=[[center[0]+length/2*math.cos(a),center[1]+width/2*math.sin(a)] for a in p.linspace(0,2*math.pi,41)]
            patches.append({"name":c,"x":[a[0] for a in pts],"y":[a[1] for a in pts]})
        plots.append({"title":"Contact patches (Fz / pressure)","xlabel":"x forward m","ylabel":"y left m","series":patches,"equal":True})
    elif key=="lap":
        from dataclasses import replace
        rf,rr=rc_values(v,geo,o)
        vv=replace(v,mu_lat=v.mu_lat*o["grip_percent"]/100,mu_long=v.mu_long*o["grip_percent"]/100)
        rows,metrics=simulate_lap(vv,track or PRESETS["示範 Autocross"],rf,rr,o["step_m"],o.get("closed",True),
            o["start_kmh"]/3.6,o.get("aero_on",True),o["torque_percent"]/100)
        for y,title,unit in (("speed_kmh","Speed","km/h"),
                ("lat_g","Lateral acceleration","g"),("long_g","Longitudinal acceleration","g"),("elevation_m","Elevation","m")):
            plots.append(plot(rows,"distance_m",[(y,title)],title,"Distance m",unit))
        plots.append(plot(rows,"distance_m",[("front_rpm","Front motors"),("rear_rpm","Rear motors")],"Motor speed","Distance m","RPM"))
        plots.append(plot(rows,"distance_m",[("front_motor_torque_Nm","Each front motor"),("rear_motor_torque_Nm","Each rear motor")],"Available motor torque","Distance m","N m / motor"))
        plots.append(plot(rows,"distance_m",[("front_wheel_torque_Nm","Each front wheel"),("rear_wheel_torque_Nm","Each rear wheel")],"Available wheel torque","Distance m","N m / wheel"))
        plots.append(plot(rows,"distance_m",[("front_ride_mm","Front"),("rear_ride_mm","Rear")],"Ride height","Distance m","mm"))
        plots.append(plot(rows,"x_m",[("y_m","Course centerline")],"Course preview","x m","y m"))
        plots[-1]["equal"]=True
    return {"rows":rows,"metrics":metrics,"plots":plots,"note":NOTES[key],**extra}
