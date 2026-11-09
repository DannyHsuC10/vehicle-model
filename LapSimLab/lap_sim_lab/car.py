"""Map workbench inputs to the original LapSim vehicle parameters."""

from functools import cached_property

import numpy as np

from .aero import ModalAeroModel
from .motor import MotorCurve
from .settings import Settings


class Car:
    def __init__(self, settings: Settings, *, preload_motor: bool = True):
        self.m = settings.mass
        self.tf = settings.front_track
        self.tr = settings.rear_track
        self.l = settings.wheelbase
        self.lf = settings.cg_from_front
        self.lr = self.l - self.lf
        self.h_cg = settings.cg_height
        self.free_radius = self.R_tire = settings.tire_radius
        self.K_tire = settings.tire_stiffness

        self.motor_path = settings.motor_path
        if preload_motor:
            self.motor_curve = MotorCurve(self.motor_path)
        self.gear_eta = settings.gear_efficiency
        self.train_rate = settings.gear_ratio
        front, rear = settings.front_torque_scale, settings.rear_torque_scale
        self.T_rate = np.array([front, front, rear, rear])

        self.brake_bias = np.array([settings.front_brake_bias, 1.0 - settings.front_brake_bias])
        self.brake_mechanical_advantage = settings.brake_leverage
        self.F_diver = settings.pedal_force

        self.k_heave_front = settings.front_heave_stiffness
        self.k_heave_rear = settings.rear_heave_stiffness
        self.k_roll_front = settings.front_roll_stiffness
        self.k_roll_rear = settings.rear_roll_stiffness
        self.aero = None
        if settings.aero_enabled:
            self.aero = ModalAeroModel.from_json(
                settings.aero_path,
                wheelbase=self.l,
                track=(self.tf + self.tr) / 2.0,
            )

    @cached_property
    def motor_curve(self):
        return MotorCurve(self.motor_path)

    def get_motor_torque_data(self, speed_rpm):
        return self.motor_curve.torque_at(speed_rpm)
