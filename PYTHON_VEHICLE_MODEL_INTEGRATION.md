# Python 車輛模型與 MPCC 車輛模型差異及連接方式

本文比較 `python/` 中的車輛模型與原本 Matlab、C++ MPCC 內建車輛模型的差異，並說明如何把 Python 車輛模型接到 Matlab 或 C++ 的「駕駛人模型」也就是 MPCC 控制器。

## 1. Python 車輛模型定位

`python/` 內的模型不是單純的 bicycle model，而是比較完整的 FSAE 四輪整車時域模型。核心入口是：

- `python/Vehicle_Dynamics.py`
- `python/Vehicle_status.py`
- `python/car.py`

其中 `VehicleModel.step(command)` 是整車模擬每一步的主要流程。它吃一個 `DriverCommand`：

```python
DriverCommand(
    throttle=0.0,       # 0 ~ 1
    brake=0.0,          # 0 ~ 1
    steer=np.zeros(4)   # [FL, FR, RL, RR], 單位為 degree
)
```

然後更新：

```text
駕駛命令
  -> 馬達/煞車輪上扭矩
  -> 四輪瞬態 Pacejka 輪胎力
  -> 輪速動力學
  -> yaw moment
  -> 空力
  -> 懸吊/輪重轉移
  -> 車身加速度與角加速度
  -> Euler 積分更新整車狀態
```

這代表 Python 模型比較適合當作「完整車輛 plant / simulator」，而 Matlab/C++ 內建 bicycle model 比較適合當作 MPCC 控制器內部的「預測模型」。

## 2. 三套車輛模型的主要差異

### 狀態維度與狀態定義

Matlab MPCC 使用 7 維狀態：

```text
x = [X, Y, phi, vx, vy, omega, theta]
u = [D, delta, vtheta]
```

C++ MPCC 使用 10 維狀態：

```text
State = [X, Y, phi, vx, vy, r, s, D, delta, vs]
Input = [dD, dDelta, dVs]
```

Python 模型狀態分散在 `Vehicle_state` 類別中，包含：

```text
s_world = [X_world, Y_world, Z_world]
v       = [vx_body, vy_body, vz_body]
a       = [ax_body, ay_body, az_body]
theta   = [front_roll, rear_roll, pitch, yaw]
omega   = [front_roll_rate, rear_roll_rate, pitch_rate, yaw_rate]
alpha   = [front_roll_acc, rear_roll_acc, pitch_acc, yaw_acc]

omega_w = [FL, FR, RL, RR] wheel speed
steer   = [FL, FR, RL, RR] steering angle in degree
Fx/Fy/Fz, SA/SL, loaded_radius, suspension states, aero states, EV states...
```

其中與 MPCC 平面控制最相關的是：

```text
X     = state.s_world[0]
Y     = state.s_world[1]
phi   = state.theta[3]
vx    = state.v[0]
vy    = state.v[1]
yaw r = state.omega[3]
```

### 輸入定義差異

Matlab MPCC 的控制輸入是：

```text
D       duty cycle / 等效油門
delta   單一前輪轉角，單位 rad
vtheta  虛擬路徑速度
```

C++ MPCC 的最佳化輸入是 input rate：

```text
dD, dDelta, dVs
```

但 C++ state 內同時保存實際命令：

```text
D, delta, vs
```

Python 模型吃的是更接近整車模擬的命令：

```text
throttle  0 ~ 1
brake     0 ~ 1
steer     四輪轉角，單位 degree
```

所以接到 Python 時要做命令轉換：

```text
D / dD         -> throttle 或 brake
delta [rad]    -> front steer [deg]
vtheta / vs    -> Python plant 不直接使用，只保留在 MPCC 內部
```

### 車輛動態模型差異

Matlab/C++ MPCC 內建模型是單軌 bicycle model：

- 只有一個前輪等效側向力、一個後輪等效側向力
- 主要描述平面運動：`X, Y, phi, vx, vy, yaw rate`
- 輪胎模型是簡化 Magic Formula：

```text
F_y = D_tire * sin(C_tire * atan(B_tire * alpha))
```

- Matlab 版驅動力為：

```text
F_rx = Cm1*D - Cm2*D*vx - Cr0 - Cr2*vx^2
```

- C++ 版將後輪驅動力與摩擦阻力拆成：

```text
F_rx   = Cm1*D - Cm2*D*vx
F_fric = -Cr0 - Cr2*vx^2
```

Python 模型是四輪模型：

- 每個輪子都有自己的 `Fx, Fy, Fz, Mx, Mz, RRT`
- 使用 Pacejka Magic Formula 6.1.2，從 `.tir` 檔載入輪胎參數
- 支援 slip angle、slip ratio、camber、load、speed、loaded radius
- 可使用瞬態輪胎模型，透過 relaxation length 建立一階延遲
- 有輪速動力學：

```text
alpha_w = (T_drive + T_brake - Fx * loaded_radius + RRT_w) / I_w
```

- 有懸吊與輪重轉移：`Suspension_model_v13.motion(state, car)`
- 有空力：`ModalAeroModel.calculate_from_state(state)`
- 有馬達扭矩表或 EV ROM 模型
- 有 traction control / brake controller

### 積分方式差異

Matlab：

- `SimTimeStep.m` 使用 `ode45`
- MPC 內部使用線性化離散模型

C++：

- `Integrator::simTimeStep()` 使用 RK4
- MPC 內部使用線性化離散模型

Python：

- `Vehicle_state.Integration()` 使用顯式 Euler 積分
- 預設 `dt = 0.001 s`
- 整車模型內部同時更新輪速、CG、姿態、world position

Python 的時間步長通常比 MPCC 控制週期小。Matlab ORCA 預設 `Ts = 0.02 s`，C++ `config.json` 中也通常是控制層較大的 `Ts`。因此接合時建議：

```text
MPCC 每 Ts 秒算一次控制命令
Python plant 在同一個命令下跑 Ts / dt_python 個小步
```

## 3. 不能直接一對一替換的原因

Python 模型比 Matlab/C++ 內建模型完整，但不適合直接塞進目前 MPCC 的 QP formulation，原因是：

1. MPCC 需要在每個 horizon stage 取得線性化離散模型 `Ad, Bd, gd`。
2. Matlab/C++ 目前都手寫了模型 Jacobian。
3. Python 模型包含四輪輪胎、輪速、懸吊、空力、控制器、查表與條件邏輯，不容易手推 Jacobian。
4. Python 模型 state 維度遠大於 MPCC 目前假設的 7 維或 10 維。
5. Python 中 steer 是四輪 degree 命令，MPCC 中 `delta` 是單軌 rad 命令。
6. Python 動力輸入是 throttle/brake，MPCC 的 `D` 是抽象 duty cycle。

因此推薦接法不是「把 Python 模型取代 `DiscretizedLinearizedModel.m` 或 `Model::getF()`」，而是：

```text
MPCC 使用原本簡化 bicycle model 作為控制器預測模型
Python 使用完整四輪模型作為外部 plant
兩者透過狀態回授與控制命令 adapter 連接
```

這種架構也比較符合實車控制：控制器內部使用可即時最佳化的簡化模型，真實車輛或高保真模擬器則作為被控對象。

## 4. 與 Matlab MPCC 連接方式

### 資料流

建議架構如下：

```text
Python VehicleModel state
  -> extract MPCC state x0
  -> Matlab optimizer_mpcc(...)
  -> get u = [D, delta, vtheta]
  -> convert u to Python DriverCommand
  -> Python VehicleModel.step(command) 多個小步
  -> 下一個控制週期
```

### Python 狀態轉 Matlab MPCC 狀態

Matlab MPCC 需要：

```matlab
x0 = [X, Y, phi, vx, vy, omega, theta]'
```

Python 對應為：

```text
X     = vehicle.state.s_world[0]
Y     = vehicle.state.s_world[1]
phi   = vehicle.state.theta[3]
vx    = vehicle.state.v[0]
vy    = vehicle.state.v[1]
omega = vehicle.state.omega[3]
theta = 用 findTheta(...) 或 MPCC 內部投影更新
```

`theta` 不應該從 Python state 直接拿，因為 Python 沒有路徑進度狀態。它應由 Matlab MPCC 根據 `(X, Y)` 投影到參考路徑取得。

### Matlab MPCC 輸出轉 Python 命令

Matlab 輸出：

```matlab
u(:,1) = [D, delta, vtheta]
```

可轉成：

```python
def matlab_u_to_python_command(u):
    D = float(u[0])
    delta_rad = float(u[1])

    throttle = max(0.0, min(1.0, D))

    # Matlab 的 D 下限常用 -0.1；若要支援煞車，可以將負 D 映射到 brake。
    brake = max(0.0, min(1.0, -D))

    delta_deg = np.degrees(delta_rad)
    steer = np.array([delta_deg, delta_deg, 0.0, 0.0])

    return DriverCommand(
        throttle=throttle,
        brake=brake,
        steer=steer
    )
```

注意 Python `VehicleModel.apply_driver_command()` 會用 `car.steer_limit_deg` 裁切轉角。Matlab/C++ 的 `delta` 是 rad；Python `steer` 是 degree。

### 控制週期範例

概念上：

```python
vehicle = VehicleModel(dt=0.001)
Ts_mpc = 0.02
n_substeps = int(round(Ts_mpc / vehicle.state.dt))

for k in range(n_control_steps):
    # 1. Python plant -> Matlab MPCC state
    x0 = extract_matlab_state(vehicle.state)

    # 2. 呼叫 Matlab MPCC，取得 u = [D, delta, vtheta]
    # u = matlab_engine.optimizer_or_wrapper(...)

    # 3. MPCC command -> Python command
    cmd = matlab_u_to_python_command(u)

    # 4. Python plant 在固定命令下跑多個小步
    for _ in range(n_substeps):
        vehicle.step(cmd)
```

真正實作時建議把 Matlab 原本 `simulation.m` 拆成「初始化」與「單步控制」兩個函式，不要讓 `simulation.m` 自己再呼叫 `SimTimeStep.m`。此時 `SimTimeStep.m` 由 Python `VehicleModel.step()` 取代。

## 5. 與 C++ MPCC 連接方式

### 資料流

C++ MPCC 的外部連接更適合做成 co-simulation：

```text
Python VehicleModel state
  -> convert to mpcc::State
  -> mpc.runMPC(x0)
  -> MPCReturn.u0 = [dD, dDelta, dVs]
  -> integrate command state D/delta/vs
  -> convert to Python DriverCommand
  -> Python VehicleModel.step(command) 多個小步
```

### Python 狀態轉 C++ MPCC 狀態

C++ `State` 需要：

```cpp
State = {X, Y, phi, vx, vy, r, s, D, delta, vs}
```

Python 對應為：

```text
X     = vehicle.state.s_world[0]
Y     = vehicle.state.s_world[1]
phi   = vehicle.state.theta[3]
vx    = vehicle.state.v[0]
vy    = vehicle.state.v[1]
r     = vehicle.state.omega[3]
s     = 由 C++ ArcLengthSpline::porjectOnSpline(x0) 投影，也可沿用上一輪
D     = adapter 內保存的上一輪等效油門
delta = adapter 內保存的上一輪單軌轉角，rad
vs    = adapter 內保存的上一輪路徑速度
```

C++ `runMPC()` 內部會呼叫：

```cpp
x0.s = track_.porjectOnSpline(x0);
```

所以 `s` 可以先給上一輪值，讓投影以它作為初始猜測。

### C++ MPCC 輸出轉 Python 命令

C++ 回傳的是 input rate：

```text
u0 = [dD, dDelta, dVs]
```

adapter 需要自己維護實際命令：

```python
class CppMpccCommandAdapter:
    def __init__(self, Ts):
        self.Ts = Ts
        self.D = 0.0
        self.delta = 0.0
        self.vs = 0.0

    def update(self, dD, dDelta, dVs):
        self.D += self.Ts * dD
        self.delta += self.Ts * dDelta
        self.vs += self.Ts * dVs

        self.D = np.clip(self.D, -1.0, 1.0)
        self.delta = np.clip(self.delta, np.radians(-34), np.radians(34))

        throttle = max(0.0, self.D)
        brake = max(0.0, -self.D)

        steer_deg = np.degrees(self.delta)
        steer = np.array([steer_deg, steer_deg, 0.0, 0.0])

        return DriverCommand(
            throttle=throttle,
            brake=brake,
            steer=steer
        )
```

如果你想避免 adapter 重複積分，也可以修改 C++ `MPCReturn`，讓它額外回傳解出的第一個 predicted state 中的 `D`、`delta`、`vs`，然後 Python 直接使用這些實際命令。

## 6. 建議的實作層級

### 最小可行接法

最小可行版本建議這樣做：

1. 保留 Matlab/C++ MPCC 內建 bicycle model，不改 QP。
2. 把 Python `VehicleModel` 當作外部 plant。
3. 每個 MPCC 控制週期從 Python state 抽出：

```text
[X, Y, phi, vx, vy, yaw_rate]
```

4. MPCC 解出控制命令。
5. 將 `delta rad` 轉成前輪 `steer deg`。
6. 將 `D` 轉成 `throttle/brake`。
7. Python plant 用較小 `dt` 跑多個 substep。

這個版本最快能驗證「MPCC 駕駛人模型能不能開 Python 完整車」。

### 中階接法：參數對齊

若最小接法能跑，再做參數對齊：

- 將 MPCC 的 `m`, `Iz`, `lf`, `lr` 對齊 Python `Car`
- 將 MPCC 的轉角限制對齊 `car.steer_limit_deg`
- 將 MPCC 的驅動命令範圍對齊 Python throttle/brake
- 將 MPCC 的速度、yaw rate、delta bounds 對齊 Python 模型合理範圍
- 用 Python 四輪模型在固定速度與固定轉角下估測等效 bicycle tire stiffness 或 Magic Formula 參數

這一步會大幅降低「控制器以為車會這樣動，但 Python plant 實際不是」的 model mismatch。

### 進階接法：用 Python 產生近似線性化模型

如果你真的想讓 MPCC 內部也使用 Python 高保真模型，可以做數值線性化：

```text
給定 x, u
  -> 呼叫 Python plant 的單步 dynamics
  -> 用 finite difference 對 x/u 做擾動
  -> 得到 Ad, Bd, gd
  -> 丟回 Matlab/C++ QP
```

但這會遇到幾個問題：

- Python state 維度很大，QP 會變重
- 懸吊、輪胎、馬達查表、控制器飽和會讓 finite difference 不平滑
- 每個 horizon stage 都要多次呼叫 Python，速度可能不適合即時控制
- 需要重寫 Matlab/C++ solver 的維度與 constraints

因此這不建議作為第一步。

## 7. 需要特別注意的座標與單位

### Steering 單位

Python `DriverCommand.steer` 最終會進入：

```python
state.steer = np.clip(steer, -car.steer_limit_deg, car.steer_limit_deg)
```

輪胎模型內部再做：

```python
steer = np.deg2rad(np.array(steer))
```

所以 adapter 傳給 Python 的 steer 必須是 degree。Matlab/C++ MPCC 的 `delta` 是 rad。

### Yaw index

Python 的 yaw 是：

```text
state.theta[3]
state.omega[3]
state.alpha[3]
```

不要誤用 `theta[2]`。在 Python 懸吊模型中 `theta[2]` 更接近 pitch。

### Body frame 與 world frame

Python 中：

- `state.v` 是 body frame velocity
- `state.s_world` 是 world frame position
- `state.v_world` 是 world frame velocity

MPCC 需要：

- world frame position：`X`, `Y`
- body frame velocity：`vx`, `vy`
- world heading：`phi`

所以應使用：

```text
X, Y = s_world[0], s_world[1]
vx, vy = v[0], v[1]
phi = theta[3]
```

### 前輪轉角

MPCC 是單軌模型，只有一個 `delta`。Python 可接受四輪轉角。最小接法可先用：

```text
steer = [delta_deg, delta_deg, 0, 0]
```

若之後要更接近實車，可以加入 Ackermann 轉換，把單一 `delta` 轉成不同的 FL/FR 轉角。

### 油門與煞車

Python 有獨立：

```text
throttle = 0 ~ 1
brake = 0 ~ 1
```

MPCC 的 `D` 在原模型中是 duty cycle，不是煞車命令。若允許 `D < 0`，可以先把負值視為 brake；但這只是工程 adapter，並不是原 MPCC 物理模型的一部分。更嚴謹的作法是修改 MPCC input，明確拆成 throttle 與 brake。

## 8. 推薦工作路線

建議依序做：

1. 寫一個 adapter，從 Python state 取出 MPCC 所需 6 個平面狀態。
2. 先用 Matlab 或 C++ MPCC 控制 Python plant，不改 MPCC 內部模型。
3. 使用固定寬度 corridor 或已知目標路徑，先避免障礙物與複雜邊界。
4. 讓 Python plant 用 `dt = 0.001`，MPCC 用 `Ts = 0.02` 或 `0.05`，每次控制命令保持一個 MPCC 週期。
5. 檢查單位：`delta rad -> steer deg`、`yaw = theta[3]`、`vx/vy = body frame`。
6. 若車輛能穩定跟路徑，再校正 MPCC bicycle model 參數，使它接近 Python 四輪模型的低頻平面響應。
7. 最後才考慮數值線性化或更完整的模型替換。

最重要的設計原則是：Python 模型先當被控車輛，MPCC 先當駕駛人。不要一開始就把 Python 高保真模型硬塞進 QP，否則會同時碰到維度、速度、Jacobian、限制式與單位轉換問題。
