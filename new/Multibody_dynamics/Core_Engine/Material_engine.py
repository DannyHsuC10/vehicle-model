# material_engine.py
# 純粹的單一桿件材料力學計算機
import numpy as np

class MaterialEngine:
    def __init__(self):
        pass

    def compute_rod_mechanics(self, r, F):
        """
        直接傳入單一連桿物件 (r) 與當前求解出的受力 (F)
        直接在物件上更新並同步所有材料力學數值。
        """
        # 1. 讀取連桿本身的物理與幾何屬性
        A = getattr(r, 'area', 1.0)       # 截面積(mm^2)
        E = getattr(r, 'E', 2.1e5)       # 楊氏模數
        Lv = r.a.pos-r.b.pos  # 桿件向量
        L0 = np.linalg.norm(Lv)  # 桿件原長

        # 防呆阻擋：避免分母為 0 導致程式崩潰
        if A < 1e-12 or E < 1e-12 or L0 < 1e-12:
            return

        # 2. 寫入/更新連桿當前的受力
        r.force = F
        # 3. 核心材料力學公式計算
        r.stress = F / A                      # 正向應力 (σ = F/A)
        r.strain = F / (A * E)                # 正向應變 (ε = σ/E)
        r.delta_L = r.strain * L0             # 桿件伸長/縮短量 (ΔL)
        r.k = (A * E) / L0                    # 桿件等效軸向剛度 (k)
        r.energy = (F**2 * L0) / (2.0 * A * E) # 應變能 (U)

        # 4. 安全係數與損壞判定
        max_stress = getattr(r, 'max_stress', 0.0)
        if max_stress > 0.0:
            r.safety_ratio = abs(r.stress) / max_stress
            r.fail = abs(r.stress) > max_stress
        else:
            r.safety_ratio = 0.0
            r.fail = False