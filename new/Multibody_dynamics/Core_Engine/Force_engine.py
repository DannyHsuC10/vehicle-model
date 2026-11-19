# ============================================================
# force_engine.py
# ============================================================
# 功能：
# - generalized force system
# - force element abstraction
# - force accumulation
# - dynamics-ready architecture
#
# 目的：
# - 不直接改 position
# - 所有輸入都是 force
# - 後續可接：
#     spring
#     damper
#     tire
#     aero
#     motor
#     control
#
# ============================================================

import numpy as np

# ============================================================
# Force Manager
# ============================================================

class ForceManager:

    def __init__(self):

        self.force_elements = []

    # ========================================================

    def add_force(self, force_element):

        self.force_elements.append(force_element)

    # ========================================================

    def compute_total_Q(self, state, n_dof):

        Q_total = np.zeros(n_dof)

        for f in self.force_elements:

            Q_total += f.compute_Q(state, n_dof)

        return Q_total
    

# ============================================================
# Base Force Element
# ============================================================

class ForceElement:

    def compute_Q(self, state, n_dof):
        raise NotImplementedError


# ============================================================
# Point Force
# ============================================================

class PointForce(ForceElement):

    def __init__(
        self,
        node,
        force
    ):

        self.node = node

        self.force = np.array(force, dtype=float)

    # ========================================================

    def set_force(self, force):

        self.force = np.array(force, dtype=float)

    # ========================================================

    def compute_Q(self, state, n_dof):

        Q = np.zeros(n_dof)

        if self.node.fixed:
            return Q

        idx = self.node.idx

        Q[idx:idx+3] += self.force

        return Q


# ============================================================
# Gravity Force
# ============================================================

class GravityForce(ForceElement):

    def __init__(
        self,
        node,
        mass,
        g=np.array([0, 0, -9810.0])
    ):

        self.node = node

        self.mass = mass

        self.g = np.array(g, dtype=float)

    # ========================================================

    def compute_Q(self, state, n_dof):

        Q = np.zeros(n_dof)

        if self.node.fixed:
            return Q

        idx = self.node.idx

        Q[idx:idx+3] += self.mass * self.g

        return Q


# ============================================================
# Spring Damper Force
# ============================================================

class SpringDamperForce(ForceElement):

    def __init__(
        self,
        node_a,
        node_b,
        k,
        c,
        L0=None
    ):

        self.a = node_a
        self.b = node_b

        self.k = k
        self.c = c

        if L0 is None:
            self.L0 = np.linalg.norm(
                self.b.pos - self.a.pos
            )
        else:
            self.L0 = L0

    # ========================================================

    def compute_Q(self, state, n_dof):

        Q = np.zeros(n_dof)

        pa = self.a.pos
        pb = self.b.pos

        va = state.get_velocity(self.a)
        vb = state.get_velocity(self.b)

        r = pb - pa

        L = np.linalg.norm(r)

        if L < 1e-12:
            return Q

        r_hat = r / L

        # ====================================================
        # spring
        # ====================================================

        spring_force = self.k * (L - self.L0)

        # ====================================================
        # damper
        # ====================================================

        rel_v = np.dot(vb - va, r_hat)

        damper_force = self.c * rel_v

        # ====================================================
        # total force
        # ====================================================

        F = (spring_force + damper_force) * r_hat

        # ====================================================
        # apply
        # ====================================================

        if not self.a.fixed:
            idx = self.a.idx
            Q[idx:idx+3] += F

        if not self.b.fixed:
            idx = self.b.idx
            Q[idx:idx+3] -= F

        return Q

