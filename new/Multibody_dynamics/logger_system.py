# auto_logger.py
import pandas as pd
import numpy as np
from pathlib import Path
from datetime import datetime
BASE_DIR = Path(__file__).resolve().parent

class AutoDataLogger:
    def __init__(self, target_nodes=None, target_rods=None):
        """
        target_nodes: 陣列或列表，指定要記錄的節點名稱。若為 None，預設自動記錄全場。
        target_rods:  陣列或列表，指定要記錄的連桿名稱。若為 None，預設自動記錄全場。
        """
        self.target_nodes = target_nodes
        self.target_rods = target_rods
        
        self.time_history = []
        self.records_history = []

    def record_all(self, sim_time, mech, state, static_analyzer, material_engine, analyzer):
        """
        自動進行前級靜力求解，並依據設定過濾或全量掃描節點與桿件數據。
        """
        self.time_history.append(sim_time)
        current_step_data = {}

        # ------------------------------------------------------------
        # 🛡️ 1. 前級計算自動化
        # ------------------------------------------------------------
        static_analyzer.solve(state)

        # ------------------------------------------------------------
        # 📍 2. 節點數據記錄 (依據篩選或全量掃描)
        # ------------------------------------------------------------
        # 判斷要掃描哪些節點：若有指定就用指定的，沒有就拿全場 mech.nodes.keys()
        nodes_to_log = self.target_nodes if self.target_nodes is not None else list(mech.nodes.keys())

        for node_name in nodes_to_log:
            node = mech.nodes.get(node_name)
            if node is None:
                continue
            
            # 位置 (pos)
            pos = getattr(node, 'pos', np.zeros(3))
            current_step_data[f"node_{node_name}_x"] = pos[0]
            current_step_data[f"node_{node_name}_y"] = pos[1]
            current_step_data[f"node_{node_name}_z"] = pos[2]
            
            # 🚀 修正：直接調用你原生的後處理器函數拿到速度與合速度大小
            vel = analyzer.get_velocity(node, state.dq)
            current_step_data[f"node_{node_name}_v"] = analyzer.get_speed(node, state.dq)
            current_step_data[f"node_{node_name}_vx"] = vel[0]
            current_step_data[f"node_{node_name}_vy"] = vel[1]
            current_step_data[f"node_{node_name}_vz"] = vel[2]
            
            # 🚀 修正：直接調用你原生的後處理器函數拿到加速度與合加速度大小
            acc = analyzer.get_acceleration(node, state.ddq)
            current_step_data[f"node_{node_name}_a"] = analyzer.get_acc_magnitude(node, state.ddq)
            current_step_data[f"node_{node_name}_ax"] = acc[0]
            current_step_data[f"node_{node_name}_ay"] = acc[1]
            current_step_data[f"node_{node_name}_az"] = acc[2]

        # ------------------------------------------------------------
        # 🥢 3. 桿件數據記錄 (依據篩選或全量掃描)
        # ------------------------------------------------------------
        # 判斷要掃描哪些桿件：若有指定就用指定的，沒有就拿全場 mech.rods.keys()
        rods_to_log = self.target_rods if self.target_rods is not None else list(mech.rods.keys())

        for rod_name in rods_to_log:
            rod = mech.rods.get(rod_name)
            if rod is None:
                continue

            # A. 運動學資料輸出
            angles = analyzer.rod_orientation_angles(rod)
            omega_vec = analyzer.rod_angular_velocity(rod, state.dq)
            alpha_vec = analyzer.rod_angular_acceleration(rod, state.dq, state.ddq)

            current_step_data[f"rod_{rod_name}_thetax"] = angles[0]
            current_step_data[f"rod_{rod_name}_thetay"] = angles[1]
            current_step_data[f"rod_{rod_name}_thetaz"] = angles[2]
            
            # 🚀 修正：直接調用你的函數拿到合角速度、合角加速度大小
            current_step_data[f"rod_{rod_name}_omega"] = analyzer.get_omega_magnitude(rod, state.dq)
            current_step_data[f"rod_{rod_name}_omegax"] = omega_vec[0]
            current_step_data[f"rod_{rod_name}_omegay"] = omega_vec[1]
            current_step_data[f"rod_{rod_name}_omegaz"] = omega_vec[2]
            
            current_step_data[f"rod_{rod_name}_alpha"] = analyzer.get_alpha_magnitude(rod, state.dq, state.ddq)
            current_step_data[f"rod_{rod_name}_alphax"] = alpha_vec[0]
            current_step_data[f"rod_{rod_name}_alphay"] = alpha_vec[1]
            current_step_data[f"rod_{rod_name}_alphaz"] = alpha_vec[2]

            # B. 材料力學求解與紀錄
            F_mag = getattr(rod, 'force', 0.0)
            material_engine.compute_rod_mechanics(rod, F_mag)

            current_step_data[f"rod_{rod_name}_F"] = F_mag
            
            # 受力 3D 空間投影分量
            d = rod.b.pos - rod.a.pos
            L = np.linalg.norm(d)
            u = d / L if L > 1e-12 else np.zeros(3)
            F_vec = F_mag * u
            current_step_data[f"rod_{rod_name}_Fx"] = F_vec[0]
            current_step_data[f"rod_{rod_name}_Fy"] = F_vec[1]
            current_step_data[f"rod_{rod_name}_Fz"] = F_vec[2]
            
            # 🚀 修正：移除了所有材料力學的單位換算（如 /1e6 或 *1000），改由你的計算函式原生輸出決定
            current_step_data[f"rod_{rod_name}_stress"] = getattr(rod, 'stress', 0.0)
            current_step_data[f"rod_{rod_name}_strain"] = getattr(rod, 'strain', 0.0)
            current_step_data[f"rod_{rod_name}_delta_L"] = getattr(rod, 'delta_L', 0.0)
            current_step_data[f"rod_{rod_name}_energy"] = getattr(rod, 'energy', 0.0)
            current_step_data[f"rod_{rod_name}_SF"] = getattr(rod, 'safety_ratio', 0.0)

        self.records_history.append(current_step_data)


class AutoOutputEngine:
    @staticmethod
    def export_to_csv(logger, decimals=4):
        """
        自動將收集到的海量歷史數據整合成單一 DataFrame，並在寫入 CSV 前自動限制小數點位數。
        
        decimals: 預設保留 4 位小數。
                  對於長度單位為 mm 的系統：
                  - .3 位 = 1 um (微米)
                  - .4 位 = 0.1 um (分微米，推薦，精準度極高且檔案極小)
        """
        if not logger.records_history:
            print("⚠️ Logger 沒有抓到任何資料。")
            return
        
        # 1. 轉換為 Pandas DataFrame
        df = pd.DataFrame(logger.records_history)
        
        # 🚀 關鍵優化 A：在寫入硬碟前，一鍵將全場數值四捨五入到指定位數
        # 這會直接砍掉小數點後十幾位的冗餘文字，讓 CSV 體積縮減 60% 以上！
        df = df.round(decimals)
        
        # 2. 插入時間軸 (時間軸可以單獨設定解析度，通常與 dt 搭配，保留 3 或 4 位即可)
        # 這裡單獨對時間序列做 round，避免受到整體設定的干擾
        rounded_time = np.round(logger.time_history, max(3, decimals))
        df.insert(0, 't', rounded_time)
        
        # 3. 輸出成 CSV

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        file_path = BASE_DIR / "output" /f"output_{timestamp}.csv"

        df.to_csv(file_path, index=False, encoding="utf-8")
        print(f"\n🎉 [數據報表導出成功] (已自動優化小數點後 {decimals} 位)")
        print(f"📊 欄位總數: {len(df.columns)} | 檔案路徑: {file_path}")