# 模擬區

from Model_Analysis import simulator
import Model_Analysis.motion as mot
import Model_Analysis.Geometry as geo
from Model_Analysis import output as out
# ========================(問題定義與求解)
class SteeringSimulation:# 模擬情況
    def __init__(self, dv_manager, x, nodes_path=None, rods_path=None):
        self.dv_manager = dv_manager
        self.x = x
        self.SIM_TIME = 30
        self.DT = 0.05
        self.nodes_path = nodes_path
        self.rods_path = rods_path

    def build(self, enable_visual=False):

        motion = mot.MotionConfig(
            motion_type="s",
            input_type="l",
            x_max=40,
            x_min=-40,
            times=1        )

        sim = simulator.SimulationSystem(enable_visual=enable_visual, nodes_path=self.nodes_path, rods_path=self.rods_path)

        # ===== 幾何 =====
        if self.dv_manager is not None and self.x is not None:
            node_dict = self.dv_manager.vector_to_dict(self.x)
            sim.override_node_positions(node_dict)

        sim.sim.prev_x = sim.solver.pack()

        # ===== input node =====
        P_input = sim.nodes["rack"]
        P_init = P_input.pos.copy()

        P_ctrl = simulator.PositionConstraint(P_input)
        sim.solver.add_constraint(P_ctrl)

        axis = geo.axis_unitization(
            sim.nodes["rack_r"].pos,
            sim.nodes["rack_l"].pos)

        # ===== input function =====
        
        def input_func(t):
            x = motion.get_input(t, self.SIM_TIME)
            new_pos = motion.apply_motion(P_init, axis, x)
            P_ctrl.set_target(new_pos)
            return x

        sim.add_input(input_func)

        wheel_axis = geo.axis_unitization(
            sim.nodes["lf_arm"].pos,
            sim.nodes["uf_arm"].pos)
        
        t_targets = [{
            "name": "wheel_torque",
            "node": "tie_rod",
            "axis": wheel_axis
        }]

        # 設定解出點
        sim.set_targets(
            node_targets=["rack", "tie_rod"],
            rod_targets=["racktie", "uAtie"],
            force_targets=[{
                "name": "rack_force",       # 力量
                "in_node": P_input,         # 作用點
                "in_axis": axis[0],         # 方向
                "in_force": 10,          # 大小
            }],
            torque_targets = t_targets      # 傳入力矩(也可以是list)
            )
        
        return sim
    
    def run(self, enable_visual=False):
        sim = self.build(enable_visual=enable_visual)
        
        sim.run(SIM_TIME=self.SIM_TIME, DT=self.DT)
        return sim, sim.success  # 更乾淨

if __name__ == "__main__":

    # 純模擬
    sim_runner = SteeringSimulation(None, None)
    sim, success = sim_runner.run(enable_visual=True)
    sim.plot(["uAtie_thetaz","rack_y","tau"], save=True)

    #out.print_out(sim,"tau")

    # 輸出
    out.save_to_csv(sim)

    """# 修改指定點比較用
    geo_provider = bd.GeometryProvider()# 直接模擬不需要用
    dv_manager = bd.DesignVariableManager(geo_provider.get_base_nodes())# 直接模擬不需要用

    # initial
    sim_init, _ = SteeringSimulation(dv_manager,[78.65,552.44,179.52]).run()

    # optimized
    sim_opt, _ = SteeringSimulation(dv_manager, [93.65,582.44,159.52]).run()

    """

