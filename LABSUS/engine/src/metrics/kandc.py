"""K&C 增益指标层：由扫掠曲线提取工业常用增益（°/25mm、°/kN、MR 矩阵、RC 迁移）。

单一实现约定（F-12/F-13，2026-08-30）：
- `_slope` 是本模块与全引擎唯一的中央差分内核（供 v3service/chassis 共用，
  删除了两处逐字拷贝）。数学 = 最近邻采样点中央差分（讲义第五讲钦定
  h 窗口的精神：噪声来自位移残差 1e-13，差分窗口取实际采样步长最稳）。
- `kinematics.py` 的同名指标为状态机包装（MetricResult），其数值内核
  委托本模块 `_slope`，两边数学恒等（F-12 双语义收敛）。
- 端点用单侧窗口（只有一个邻点时用该侧差分），不再放大端点增益。

K&C gain metric layer: extracts industry-standard gains from sweep curves
(deg/25mm, deg/kN, the MR matrix, RC migration).

Single-implementation convention (F-12/F-13, 2026-08-30):
- `_slope` is the one and only central-difference kernel in this module
  and across the engine (shared by v3service/chassis; two verbatim copies
  were deleted). The math = central difference over the nearest-neighbor
  sampling points (in the spirit of the lecture-5-sanctioned h window -
  noise comes from the 1e-13 displacement residual, so a diff window
  equal to the actual sampling step is the most robust).
- The same-named metrics in `kinematics.py` are state-machine wrappers
  (MetricResult); their numeric kernel delegates to this module's
  `_slope`, and the math on both sides is identical (F-12 dual-semantic
  convergence).
- Endpoints use a one-sided window (when there is only one neighboring
  point, use that side's difference); endpoint gains are no longer
  exaggerated.
"""

# F-10（讲义 A.2 修订）：力轴差分不适用固定 ±2N 窗口（力噪声量级与位移不同），
# 统一走最近邻中央差分（实际步长），由 `window` 参数显式控制期望窗口宽度。
# F-10 (lecture A.2 revision): force-axis differencing does not fit a fixed
# ±2N window (force noise is on a different scale from displacement);
# uniformly use the nearest-neighbor central difference (actual step size),
# with the `window` parameter explicitly controlling the desired window
# width.
def _slope(y: list, x: list, at: float) -> float:
    """最近邻采样点中央差分 dy/dx @ at。

    - 有效点：自动跳过 None（扫掠中偶发求解失败的曲线点）。
    - 取 at 两侧最近的两个**有效**采样点做差商；单侧不足时退化为单侧
      差分（端点不再虚高一倍）。少于 2 个有效点 → 0.0。

    Nearest-neighbor sampling-point central difference dy/dx @ at.

    - Valid points: None is skipped automatically (curve points whose
      solve occasionally failed during a sweep).
    - Takes the two nearest **valid** sampling points on either side of
      at for the difference quotient; when one side is missing, fall back
      to a one-sided difference (endpoints are no longer artificially
      doubled). Fewer than 2 valid points → 0.0.
    """
    res = slope_at_travel(y, x, at)
    return res[0] if res is not None else 0.0


def camber_gain_deg_per_25(camber: list, travel: list, at: float) -> float:
    return round(25.0 * _slope(camber, travel, at), 6)


def bump_steer_deg_per_25(toe: list, travel: list, at: float) -> float:
    return round(25.0 * _slope(toe, travel, at), 6)


def compliance_toe_deg(toe: list, force: list, at: float) -> float:
    """单位 °/1000N（对力曲线斜率，最近邻中央差分——F-10 不再用 ±2N 窗口）。

    Units deg/1000N (slope against the force curve, nearest-neighbor
    central difference — F-10: the ±2N window is no longer used).
    """
    return round(1000.0 * _slope(toe, force, at), 6)


def mr_matrix(damper_travel: list, wheel_travel: list, at: float) -> float:
    """d(DamperTravel)/d(WheelTravel)（导数矩阵对角线项，S1 每轮单值）。

    ⚠️ 生产零引用：整车 MR 走 `chassis` 的 `mr_at_zero` 与前端扫掠。本函数保留为
    **F-12「差分内核单一实现」门禁的独立参考实现**——`test_w4_consistency.py` 用它
    逐位对齐 legacy `kinematics.motion_ratio`，证明状态机版确实委托同一个 `_slope`。
    不要把本函数接进生产路径（会多出一条 MR 口径）。

    d(DamperTravel)/d(WheelTravel) (the derivative-matrix diagonal term,
    a single value per wheel in S1).

    ⚠️ Zero production references: the vehicle-level MR goes through
    `chassis`'s `mr_at_zero` and the frontend sweep. This function is
    kept as the **standalone reference implementation for the F-12
    "single central-difference kernel" gate** — `test_w4_consistency.py`
    uses it to verify bit-identical agreement with legacy
    `kinematics.motion_ratio`, proving the state-machine version really
    does delegate to the same `_slope`. Do not wire this function into
    the production path (it would add a second MR convention).
    """
    return round(_slope(damper_travel, wheel_travel, at), 6)


# ── F-12：kinematics.py 状态机版本的数值内核（单一实现） ──────────
# F-12: the numeric kernel for the kinematics.py state-machine version (single implementation)
def slope_at_travel(curve: list, travel: list, at_travel: float):
    """供 metrics/kinematics.py 委托的差商接口：返回 (slope, i_lo, i_hi) 或 None。

    None 曲线点自动跳过；两侧最近有效点对做中央差分；单侧退化为单侧差分。

    Difference-quotient interface for metrics/kinematics.py to delegate
    to: returns (slope, i_lo, i_hi) or None.

    None curve points are skipped automatically; the nearest valid point
    pair on both sides is used for the central difference; with one side
    missing it degrades to a one-sided difference.
    """
    valid = [(i, float(v)) for i, v in enumerate(curve) if v is not None]
    if len(valid) < 2:
        return None
    below = [p for p in valid if float(travel[p[0]]) <= at_travel]
    above = [p for p in valid if float(travel[p[0]]) >= at_travel]
    lo = below[-1] if below else valid[0]
    hi = above[0] if above else valid[-1]
    if lo[0] == hi[0]:
        lo_idx = valid.index(lo)
        if lo_idx > 0:
            lo = valid[lo_idx - 1]
        elif lo_idx < len(valid) - 1:
            hi = valid[lo_idx + 1]
        else:
            return None
    dx = float(travel[hi[0]]) - float(travel[lo[0]])
    if abs(dx) < 1e-12:
        return None
    return (hi[1] - lo[1]) / dx, lo[0], hi[0]
