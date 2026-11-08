"""逐轮转速自由度测试（M18 / G48）

把状态扩容的**验收标准**先钉成断言（R-0931 计划步骤 1）：
自由滚动应收敛到纯滚动、驱动扭矩下稳态滑移率应与解析解一致、制动能抱死且不反向自转、
惯量口径与前端一致。

English: per-wheel spin-DOF tests (M18 / G48).
The acceptance criteria for the state expansion are pinned as assertions up front (R-0931 plan step 1): free rolling converges
to pure roll, steady slip under drive torque matches the analytic solution, braking locks the wheel without reverse spin, and
the inertia convention agrees with the frontend.
"""

from __future__ import annotations

import pytest

from src.solver.wheel_dof import V_MIN, WheelDof, steady_slip_ratio, wheel_inertia

R = 0.23          # FSAE 前胎半径 m — FSAE front-tire radius in m
IW = 0.35         # FSAE 单轮转动惯量（= wheel_inertia(235)） — FSAE single-wheel inertia (= wheel_inertia(235))


def _integrate(w: WheelDof, vx: float, c_k: float, t_drive: float, n: int, dt: float,
               t_brake: float = 0.0) -> WheelDof:
    """线性轮胎 Fx = Cκ·κ 下推进 n 步（κ 每步由当前 ω 算出）。"""  # English: advance n steps under the linear tire Fx = Cκ·κ (κ recomputed from the current ω each step).
    for _ in range(n):
        fx = c_k * w.slip_ratio(vx)
        w = w.advance(t_drive, t_brake, fx, dt)
    return w


def test_inertia_matches_frontend_formula():
    """惯量口径必须与前端同式：max(0.35, 1.2·m/1250)。"""  # English: the inertia convention must use the same formula as the frontend: max(0.35, 1.2·m/1250).
    assert wheel_inertia(235.0) == pytest.approx(0.35, rel=1e-12)      # FSAE：取地板 0.35 — FSAE: hits the 0.35 floor
    assert wheel_inertia(1250.0) == pytest.approx(1.2, rel=1e-12)      # GT3
    assert wheel_inertia(3000.0) > wheel_inertia(1250.0) > 0.0


def test_free_rolling_converges_to_pure_roll():
    """自由滚动（无驱动/无制动）应从任意初始滑移收敛到纯滚动 κ→0（验收标准 ①）。"""  # English: free rolling (no drive / no brake) must converge from any initial slip to pure roll κ→0 (acceptance criterion ①).
    w = WheelDof(omega=1.3 * (20.0 / R), radius_m=R, inertia_kg_m2=IW)   # 初始 κ≈0.3 — initial κ≈0.3
    assert w.slip_ratio(20.0) > 0.25
    w = _integrate(w, 20.0, c_k=60000.0, t_drive=0.0, n=4000, dt=0.002)   # 8 s
    assert abs(w.slip_ratio(20.0)) < 0.02, f"自由滚动应收敛到纯滚动（κ={w.slip_ratio(20.0):.4f}）"
    assert w.omega == pytest.approx(w.free_roll_omega(20.0), rel=0.02)


def test_steady_slip_matches_analytic_solution():
    """驱动扭矩稳态滑移率必须与解析解 κ = T/(Cκ·Re) 一致（验收标准 ②）。"""  # English: steady-state slip under drive torque must match the analytic κ = T/(Cκ·Re) (acceptance criterion ②).
    c_k, t_drive, vx = 60000.0, 400.0, 20.0
    target = steady_slip_ratio(t_drive, c_k, R)
    assert target > 0.02, "本用例的稳态滑移率应显著非零，否则测不出东西"
    w = WheelDof(omega=vx / R, radius_m=R, inertia_kg_m2=IW)
    w = _integrate(w, vx, c_k, t_drive, n=6000, dt=0.002)                 # 12 s
    assert w.slip_ratio(vx) == pytest.approx(target, rel=0.05), (
        f"稳态滑移率 {w.slip_ratio(vx):.4f} 应≈解析解 {target:.4f}")


def test_driven_wheel_has_reflected_inertia():
    """驱动轮承担传动链反射惯量，非驱动轮不承担（与前端 iwEff 口径一致）。"""  # English: driven wheels carry the driveline reflected inertia; non-driven wheels do not (same convention as the frontend iwEff).
    base = WheelDof(omega=10.0, radius_m=R, inertia_kg_m2=IW, reflected_kg_m2=0.8)
    assert base.effective_inertia == pytest.approx(IW, rel=1e-12)
    assert base.advance(100.0, 0.0, 0.0, 1.0).omega == pytest.approx(
        WheelDof(omega=10.0, radius_m=R, inertia_kg_m2=IW).advance(100.0, 0.0, 0.0, 1.0).omega, rel=0.3)
    driven = WheelDof(omega=10.0, radius_m=R, inertia_kg_m2=IW, reflected_kg_m2=0.8, driven=True)
    assert driven.effective_inertia == pytest.approx(IW + 0.8, rel=1e-12)
    # 同样扭矩下，惯量更大 ⇒ 角加速度更小 — with the same torque, larger inertia ⇒ smaller angular acceleration
    assert abs(driven.omega_dot(100.0, 0.0, 0.0)) < abs(base.omega_dot(100.0, 0.0, 0.0))


def test_brake_locks_wheel_and_never_reverses_spin():
    """制动应能抱死（ω→0 且不反向自转），滑移率随之到 −1 附近。

    ⚠ 必须用**带饱和**的轮胎力：线性 `Fx=Cκ·κ` 在 κ=−1 时给出 −60000N（远超 μFz≈1200N），
    地面反力矩会把轮重新"甩"起来 ⇒ 永远抱不死。初版测试就栽在这里，而它暴露的正是
    "轮自由度必须配饱和轮胎力"这一真实约束。

    English: braking must lock the wheel (ω→0 without reverse spin), with the slip ratio approaching −1.
    WARNING: the tire force must be saturated: the linear `Fx=Cκ·κ` gives −60000N at κ=−1 (far beyond μFz≈1200N) and the ground
    reaction torque would fling the wheel back up ⇒ it would never lock. The first-draft test failed exactly here, and what it
    exposed is a real constraint: the wheel DOF must be paired with a saturated tire force.
    """
    mu_fz, c_k, vx = 1200.0, 60000.0, 20.0

    def fx_of(w):
        k = w.slip_ratio(vx)
        return max(-mu_fz, min(mu_fz, c_k * k))

    w = WheelDof(omega=vx / R, radius_m=R, inertia_kg_m2=IW)
    for _ in range(600):                       # 1.2 s
        w = w.advance(0.0, 1500.0, fx_of(w), 0.001)
    assert w.omega >= 0.0, "制动不得把轮拖成反向自转"
    assert w.omega == pytest.approx(0.0, abs=1e-9), f"制动矩应抱死（ω={w.omega}）"
    assert w.slip_ratio(vx) == pytest.approx(-1.0, abs=0.02), "抱死后滑移率应≈−1"


def test_step_size_constraint_is_speed_dependent():
    """**计划级发现（修正版）**：轮自由度的有效时间常数是

        τ_eff = I·vx / (Cκ·Re²)

    它**随车速变化**（FSAE 实测：≈0.55ms @5m/s、≈2.2ms @20m/s）。而引擎子步约 1.6ms
    ⇒ 中低速区 **dt ≳ τ** ⇒ 显式积分临界甚至发散 ⇒ **接线前必须先定积分策略**
    （隐式 / ω 准稳态代数解 / 轮状态微子步），不能直接接。

    ⚠ 初版我写成 `τ = I/Cκ ≈ 5.8µs` —— **量纲对但不是控制常数**：κ 已按 vx 归一化，
    真实的每单位 ω 刚度是 `Cκ·Re²/vx`，故 τ 含 vx。测试当场把这条纠正了。

    English: planning-level finding (corrected version): the effective time constant of the wheel DOF is
        τ_eff = I·vx / (Cκ·Re²)
    and it varies with speed (FSAE measured: ≈0.55ms @5m/s, ≈2.2ms @20m/s). Engine substeps are about 1.6ms ⇒ in the mid/low-speed
    region dt ≳ τ ⇒ explicit integration is marginal or divergent ⇒ an integration strategy (implicit / quasi-steady algebraic ω
    solution / micro-substeps for wheel state) must be settled before wiring — it cannot be plugged in directly.
    WARNING: the first draft wrote `τ = I/Cκ ≈ 5.8µs` — dimensionally right but not the governing constant: κ is already
    normalized by vx, so the real stiffness per unit ω is `Cκ·Re²/vx` and τ therefore contains vx. The test corrected this on the spot.
    """
    c_k = 60000.0
    def tau(vx):
        return IW * vx / (c_k * R * R)
    assert tau(20.0) == pytest.approx(2.2e-3, rel=0.2), f"20m/s 时 τ={tau(20.0):.2e}s"
    assert tau(3.0) < 4e-4, f"3m/s 时 τ={tau(3.0):.2e}s（低速更刚）"
    assert tau(3.0) < tau(20.0) * 0.25, "τ 正比于 vx：低速更刚"
    # 低速区 dt=2ms ≫ τ ⇒ 显式欧拉发散（证明不是纸面推断） — low-speed region: dt=2ms ≫ τ ⇒ explicit Euler diverges (proof, not merely a paper inference)
    vx = 3.0
    w = WheelDof(omega=1.01 * (vx / R), radius_m=R, inertia_kg_m2=IW)
    dev0 = abs(w.omega - vx / R)
    for _ in range(15):
        w = w.advance(0.0, 0.0, c_k * w.slip_ratio(vx), 0.002)
    dev1 = abs(w.omega - vx / R)
    assert dev1 > dev0 * 10.0, f"低速区 dt≫τ 应发散（{dev0:.4f} → {dev1:.4f}）"


def test_slip_sign_conventions():
    """符号约定：ω 快于纯滚动 ⇒ κ>0（驱动滑转）；慢 ⇒ κ<0（制动滑移）；相等 ⇒ 0。"""  # English: sign convention: ω faster than pure roll ⇒ κ>0 (drive spin); slower ⇒ κ<0 (brake slip); equal ⇒ 0.
    w = WheelDof(omega=0.0, radius_m=R, inertia_kg_m2=IW)
    assert w.slip_ratio(10.0) == pytest.approx(-1.0, abs=1e-12)
    assert WheelDof(omega=10.0 / R, radius_m=R).slip_ratio(10.0) == pytest.approx(0.0, abs=1e-12)
    assert WheelDof(omega=1.2 * 10.0 / R, radius_m=R).slip_ratio(10.0) > 0.0
    # 静止且不转 ⇒ κ=0（分母取下限，0/0.5=0）；原地空转 ⇒ κ>0 — stationary and not spinning ⇒ κ=0 (floored denominator, 0/0.5=0); stationary burnout ⇒ κ>0
    assert WheelDof(omega=0.0, radius_m=R).slip_ratio(0.0) == pytest.approx(0.0, abs=1e-12)
    assert WheelDof(omega=5.0, radius_m=R).slip_ratio(0.0) > 1.0, "原地空转滑移率应很大"
    assert V_MIN == 0.5


def test_zero_inertia_is_guarded():
    """惯量为零不得产生 inf/nan（退化输入必须被挡住）。"""  # English: zero inertia must not produce inf/nan (degenerate inputs must be blocked).
    w = WheelDof(omega=1.0, radius_m=R, inertia_kg_m2=0.0)
    assert w.omega_dot(100.0, 0.0, 0.0) == 0.0
