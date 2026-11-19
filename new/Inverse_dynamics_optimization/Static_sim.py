# Static_sim.py
# 靜態模擬的東東
from Model_Analysis.Inverse_dynamics import Multibody_engine as mb
from Model_Analysis.Inverse_dynamics import Static_engine as se
from pathlib import Path
from vpython import *
from Model_Analysis import simulator_static
from Model_Analysis.UI import ui_loader as uil
FILTER_DIR = Path(__file__).resolve().parent

def read_json_static(path):# 這個函式不能移除這很重要(啟動單獨顯示就會用到)
    json = uil.load_json(path)
    
    files = json["static"]["files"]
    nodes = se.load_nodes(
        files["nodes_path"],
        mb.Node
    )

    rods = se.load_rods(
        files["rods_path"],
        nodes,
        mb.Rod
    )

    return nodes, rods
# ===============================================

if __name__ == "__main__":# 顯示+靜力求解
    # 建立影像區域
    scene = se.canvas(title="Static Viewer")

    cfg = uil.ConfigLoader(FILTER_DIR/"data/test_set.json")

    runner = simulator_static.StaticSimulationRunner(
        cfg=cfg)

    state = runner.run()
    #print(state.state)# 指定結果印出

    # =======================(顯示)

    # 直接顯示幾何（不求解）
    se.show_structure(
        runner.visual_nodes,
        runner.visual_rods
    )

    # =========================
    # VPython 視窗設定
    # =========================
    scene.width = 1600
    scene.height = 800
    scene.autoscale = False
    while True:
        rate(60)


"""# 單獨使用時開放

if __name__ == "__main__":# 單純顯示
    # 建立影像區域
    print("show")
    scene = se.canvas(title="Static Viewer")

    nodes, rods = read_json_static(FILTER_DIR/"data/steer_set.json")

    # 直接顯示幾何（不求解）
    se.show_structure(nodes, rods)

    # =========================
    # VPython 視窗設定
    # =========================
    scene.width = 1600
    scene.height = 800
    scene.autoscale = False
    while True:
        rate(60)
"""


