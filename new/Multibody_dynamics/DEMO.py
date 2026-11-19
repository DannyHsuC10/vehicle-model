from vpython_renderer import VPythonRenderer
from Solver.mechanism_state import MechanismState
from Solver.dynamic_solver import DynamicSolver

from Core_Engine.Force_engine import (
    ForceManager,
    PointForce,
    SpringDamperForce
)
from vpython import rate, scene
from Core_Engine.Kinematics_engine import KinematicsAnalyzer
from Core_Engine.Multibody_engine_v3 import (
    MechanismSolver)
from loader import load_mechanism_data
from Core_Engine.Virtual_work_engine import VirtualWorkEngine
from Core_Engine.Static_engine import StaticAnalyzer # 剛剛寫好的靜力分析器

from Core_Engine.Material_engine import MaterialEngine
from pathlib import Path
# ============================================================
# Paths
# ============================================================
print("正在載入機構資料...")
BASE_DIR = Path(__file__).resolve().parent

nodes_path = BASE_DIR / "data" / "nodes.csv"
rods_path = BASE_DIR / "data" / "rods.csv"
wheels_path = BASE_DIR / "data" / "wheel.csv"


# ============================================================
# Load mechanism
# ============================================================

mech = load_mechanism_data(
    nodes_path,
    rods_path,
    wheels_path
)

# ============================================================
# State
# ============================================================
print("正在初始化機構狀態與求解器...")
solver = MechanismSolver(mech)# 機構求解器，負責建立雅可比矩陣與約束方程
print("機構狀態初始化完成，正在建立渲染器...")
renderer = VPythonRenderer(mech)# 渲染器，負責將機構狀態轉換為 VPython 的 3D 圖形展示
print("機構求解器初始化完成，正在建立機構狀態...")
state = MechanismState(solver)# 機構狀態，負責儲存節點位置、速度等資訊
print("渲染器初始化完成，正在建立分析器...")
analyzer = KinematicsAnalyzer(solver)# 運動學分析器，負責計算連桿的運動學參數（如長度、速度、加速度、角度等）
print("運動學分析器初始化完成，正在建立虛功引擎...")
vw_engine = VirtualWorkEngine(solver, analyzer)# 虛功引擎，負責計算機械利益與估算輸出力
print("虛功引擎初始化完成，正在建立材料力學引擎...")
material_engine = MaterialEngine()# 材料力學引擎，負責計算連桿的受力、應力、應變等材料力學參數
print("材料力學引擎初始化完成，正在建立力管理器...")
forces = ForceManager()# 力管理器，負責統一管理所有外力（如點力、彈簧阻尼力等），並在每個時間步提供給動力學求解器使用
print("機構資料載入完成，開始模擬...")
# ============================================================
# 🚀 自動從 CSV 資料中生成彈簧阻尼元件
# ============================================================
for rod_name, rod_data in mech.rods.items():
    if rod_data.k > 0.0 or rod_data.c > 0.0:
        # 建立彈簧阻尼力元件
        sd_force = SpringDamperForce(
            node_a=rod_data.a,
            node_b=rod_data.b,
            k=rod_data.k,
            c=rod_data.c,
            L0=None # 預設使用 CSV 兩點初始距離作為彈簧自由長度
        )
        forces.add_force(sd_force)
        print(f"成功掛載彈簧阻尼桿件: {rod_name} (k={rod_data.k}, c={rod_data.c})")
# ============================================================
# Input force
# ============================================================

ul_arm_force = PointForce(
    node=mech.nodes["ul_arm"],
    force=[0, 0, 1500]
)

forces.add_force(ul_arm_force)

ur_arm_force = PointForce(
    node=mech.nodes["ur_arm"],
    force=[0, 0, 1500]
)

forces.add_force(ur_arm_force)

# ============================================================
# Dynamics solver
# ============================================================

dyn = DynamicSolver(
    mechanism_solver=solver,
    state=state,
    force_manager=forces
)
static_analyzer = StaticAnalyzer(mech, forces, dynamic_solver=dyn)# 靜力分析器，負責在每個時間步求解靜力平衡，並將結果用於材料力學計算
# ============================================================
# Mass assignment
# ============================================================

for node in mech.nodes.values():
    # 直接將 CSV 讀進來的節點質量分配給動力學求解器
    dyn.set_node_mass(node, node.m)

# ============================================================
# Simulation loop
# ============================================================

scene.width = 1600
scene.height = 900
scene.autoscale = False

from logger_system import AutoDataLogger, AutoOutputEngine
logger = AutoDataLogger()

# ============================================================
# Simulation loop
# ============================================================
dt = 0.05
sim_time = 0.0
t_max = 10.0

while sim_time <= t_max:
    rate(100)
    
    # 1. 物理引擎時間步推進
    dyn.step(dt)
    #"""
    # 🚀 2. 一鍵全自動化：把所有工具塞進去，它會自動解靜力、算材料、掃描全場
    logger.record_all(
        sim_time=sim_time,
        mech=mech,
        state=state,
        static_analyzer=static_analyzer,
        material_engine=material_engine,
        analyzer=analyzer
    )
    #"""
    # 3. 畫面更新
    renderer.update()
    sim_time += dt

# ============================================================
# 🏁 模擬結束：自動出資料
# ============================================================
AutoOutputEngine.export_to_csv(logger)



