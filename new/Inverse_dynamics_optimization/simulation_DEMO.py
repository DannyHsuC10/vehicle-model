# 示範模擬區(轉向系統)
from Model_Analysis import simulator
import Model_Analysis.motion as mot
import Model_Analysis.Geometry as geo
import numpy as np
from Model_Analysis import output as out

# ===== 初始化 ==================================================================
# 重新設定點位置(csv 預設)
SIM_TIME = 10

sim = simulator.SimulationSystem(enable_visual=True,
                                nodes_path=None,
                                rods_path=None)

# 就放在這裡（最關鍵）
#sim.override_node_positions({
#    "tie_rod": [25,-15,12.5]})# <<亂改動有可能出錯

# 很重要：同步 solver 初始狀態
sim.sim.prev_x = sim.solver.pack()

# ===== input ============================================================================
# 輸入節點

P_input = sim.nodes["rack"]
P_input2 = sim.nodes["lf_arm"]
P_init = P_input.pos.copy()
P_init2 = P_input2.pos.copy()
# 使用 PositionConstraint 把它「鎖死」在你的運動軌跡上
P_ctrl = simulator.PositionConstraint(P_input)
P_ctrl2 = simulator.PositionConstraint(P_input2)
sim.solver.add_constraint(P_ctrl)
sim.solver.add_constraint(P_ctrl2)

# 建立軸
axis = geo.axis_unitization(
    sim.nodes["rack_r"].pos,
    sim.nodes["rack_l"].pos)

motioninput = mot.MotionConfig(
    motion_type="s",
    input_type="l",
    x_max=40,
    x_min=-40,
    times=SIM_TIME/10
)

def input_func1(t):# 轉向輸入
    x = motioninput.get_input(t, SIM_TIME)
    new_pos = motioninput.apply_motion(P_init, axis, x)
    P_ctrl.set_target(new_pos)
    return x


# 輸入2(如果需要最佳化運動頻率需要錯開)
axis2 = geo.axis_unitization(
    sim.nodes["lf_arm1fix"].pos,
    sim.nodes["lf_arm2fix"].pos)

# 面移動軸
normal_vector = np.array([0, 0, 1])

motioninput2 = mot.MotionConfig(
    motion_type="f",
    input_type="l",
    x_max=25,
    x_min=-25,
    times=SIM_TIME/10,
    normal_vector=normal_vector
)

def input_func2(t):# 懸吊輸入
    x = motioninput2.get_input(t, SIM_TIME)
    new_pos = motioninput2.apply_motion(P_init2, axis2, x)
    P_ctrl2.set_target(new_pos)
    return x

sim.add_input(input_func1)
#sim.add_input(input_func2)

# 力量設定
f_targets = [{
    "name": "rack_force",       # 力量
    "in_node": P_input,         # 作用點
    "in_axis": axis[0],         # 方向
    "in_force": 950,          # 大小
}]

wheel_axis = geo.axis_unitization(
    sim.nodes["lf_arm"].pos,
    sim.nodes["uf_arm"].pos)

t_targets = [{
    "name": "wheel_torque",
    "node": "tie_rod",
    "axis": wheel_axis
}]

# ===== targets(求解目標) ================================================================================
# 在名單上才紀錄求解，否則不算避免佔用資源

sim.set_targets(
    node_targets=["rack", "tie_rod","uf_arm"],
    rod_targets=["w", "uAtie"],
    force_targets = f_targets,       # 傳入力學配置(也可以是list)
    torque_targets = t_targets      # 傳入力矩(也可以是list)
    )

# ===== run =====
# 運動設定(總時間、步長)
sim.run(SIM_TIME=SIM_TIME, DT=0.05)

# ===== 輸出 =====
# 繪圖
#sim.plot(["uAtie_thetaz","rack_y"])
#sim.plot(["uAtie_omegaz","uAtie_thetaz"])
#sim.plot("tie_rod_fx", save=False)


# 印出資料
#out.print_out(sim,"tau")

# 輸出
out.save_to_csv(sim)


