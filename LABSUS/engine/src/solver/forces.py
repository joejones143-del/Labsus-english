"""二力杆 + 球铰静力层（S1）：由接地点外载荷求解杆件轴向力与内点合力。

约定：杆件为二力杆（轴向拉压力）；球铰 3 向反力、不传纯力矩（沿用 V1 设计 §4.4）。
超静定时用最小范数最小二乘并在结果标注 APPROXIMATE（自重残差）。

Two-force-member + ball-joint static layer (S1): solves member axial
forces and inner-point resultant forces from contact-patch external
loads.

Conventions: members are two-force members (axial tension/compression
only); ball joints provide 3-direction reactions and transmit no pure
torque (per the V1 design §4.4). For statically indeterminate cases a
minimum-norm least-squares solution is used and the result is marked
APPROXIMATE (self-weight residual).
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class LinkForce:
    a: np.ndarray
    b: np.ndarray
    id: str
    mag: float = 0.0

    def unit(self) -> np.ndarray:
        d = self.b - self.a
        n = float(np.linalg.norm(d))
        return d / n if n > 1e-12 else np.zeros(3)


def _solve_linear(A: np.ndarray, rhs: np.ndarray) -> tuple[np.ndarray, float]:
    """求解 A m = rhs；超静定取最小范数解并返回残差范数。

    Solve A m = rhs; for statically indeterminate systems take the
    minimum-norm solution and return the residual norm.
    """
    if A.shape[0] == A.shape[1]:
        try:
            m = np.linalg.solve(A, rhs)
            return m, float(np.linalg.norm(A @ m - rhs))
        except np.linalg.LinAlgError:
            pass
    m, *_ = np.linalg.lstsq(A, rhs, rcond=None)
    return m, float(np.linalg.norm(A @ m - rhs))


# ---------- 公开别名 — public alias ----------------------------------------------------------

solve_linear = _solve_linear
"""公开求解器入口；内部仍为 _solve_linear。

Public solver entry point; internally it is still _solve_linear.
"""

# ---------- T5: 接地点外载 → 锚点合力 — T5: contact-patch load → anchor resultants  ---------


@dataclass
class QSLoad:
    """接地点准静态外载（N）：fx 纵向 / fy 侧向 / fz 垂向 + 三力矩（S1 力矩占位）。

    Contact-patch quasi-static external load (N): fx longitudinal /
    fy lateral / fz vertical + three torques (S1 torque placeholder).
    """
    fx: float = 0.0
    fy: float = 0.0
    fz: float = 0.0
    mx: float = 0.0
    my: float = 0.0
    mz: float = 0.0


@dataclass
class CornerLoadsResult:
    """角点静力结果。

    Attributes:
        anchor_loads: 键 = 杆 id（车身端），值 = 3-向合力（N）。
        residual: 超静定最小二乘残差（=0 时精确闭合）。
        moment_residual: 外力矩折算到轮心后的残余范数（N·mm）。
            球铰模型（二力杆只传力）无法平衡力矩；非零时结果为 APPROXIMATE
            （F-64：力矩输入不再被静默忽略，残差显式上报）。

    Corner statics result.

    Attributes:
        anchor_loads: key = member id (chassis end), value = 3-direction
            resultant force (N).
        residual: the statically-indeterminate least-squares residual
            (=0 means an exact closure).
        moment_residual: residual norm after the external moments are
            translated to the wheel center (N·mm).
            The ball-joint model (two-force members transmit forces only)
            cannot balance moments; when non-zero the result is
            APPROXIMATE (F-64: moment inputs are no longer silently
            ignored; the residual is reported explicitly).
    """
    anchor_loads: dict[str, np.ndarray]
    residual: float
    moment_residual: float = 0.0


def _chassis_end(link: LinkForce, hub: np.ndarray) -> np.ndarray:
    """识别杆的车身端：距 hub_point 较远的一侧。

    退化情况：若两端等距（杆中点恰在 hub），默认返回 link.b；实际悬架不会出现。

    Identify the member's chassis end: the side farther from hub_point.

    Degenerate case: if the two ends are equidistant (the member midpoint
    lands exactly on the hub), it defaults to link.b; real suspension
    geometries never hit this.
    """
    da = float(np.linalg.norm(link.a - hub))
    db = float(np.linalg.norm(link.b - hub))
    return link.a if da > db else link.b


def corner_to_anchor_loads(
    q: QSLoad,
    links: list[LinkForce],
    hub_point: np.ndarray,
    cp_rel: np.ndarray,
) -> CornerLoadsResult:
    """接地点力 → 转向节静力平衡 → 各杆轴向力 → 车身内锚点合力。

    约定：
    - 杆件为二力杆（轴向拉压，mag > 0 拉伸）。
    - 球头 3 向反力平衡。
    - 超静定时取最小范数解（residual > 0 表示结果为 APPROXIMATE）。

    注意：本函数会覆写每根 LinkForce 的 ``mag`` 属性。
    调用方若需复用同一 LinkForce 实例跨角点，须自行 copy。

    F-64（2026-08-30）：CaseLoad 的力矩不再被 `_ = cp_rel` 静默丢弃——
    接地点力矩折算到轮心（刚体等效 m_hub = m_cp + r × F，r = cp_rel），
    残余经 `moment_residual` 显式上报；球铰模型无法平衡力矩，非零残余
    意味着结果是 APPROXIMATE（调用方据此降级状态并附说明）。

    Args:
        q: 接地点准静态外载荷（N / N·mm）。
        links: 汇交于球头的二力杆列表。
        hub_point: 球头（轮心）坐标。
        cp_rel: 接地点相对轮心的矢量（由运动学姿态给出）。

    Returns:
        CornerLoadsResult：锚点合力字典 + 力残差 + 力矩残余。

    T8 集成约定：anchor_loads 值为杆件施加在车身锚点上的力（即 solve_compliance
    中 f_ext 的 loads[n]），符号无需翻转，直接传入。

    Contact-patch force → knuckle static equilibrium → member axial
    forces → chassis-side anchor resultants.

    Conventions:
    - Members are two-force members (axial tension/compression;
      mag > 0 = tension).
    - Ball joints balance with 3-direction reactions.
    - For statically indeterminate cases take the minimum-norm solution
      (residual > 0 means the result is APPROXIMATE).

    Note: this function overwrites each LinkForce's ``mag`` attribute.
    Callers that reuse the same LinkForce instances across corners must
    copy them.

    F-64 (2026-08-30): the CaseLoad torques are no longer silently
    dropped via `_ = cp_rel` — contact-patch moments are translated to
    the wheel center (rigid-body equivalent m_hub = m_cp + r × F, with
    r = cp_rel) and the residual is reported explicitly via
    `moment_residual`; the ball-joint model cannot balance moments, so a
    non-zero residual means the result is APPROXIMATE (the caller should
    downgrade the status accordingly and attach a note).

    Args:
        q: the quasi-static external load at the contact patch (N / N·mm).
        links: the two-force members meeting at the ball joint.
        hub_point: the ball-joint (wheel-center) coordinates.
        cp_rel: the contact-patch position relative to the wheel center
            (given by the kinematic posture).

    Returns:
        CornerLoadsResult: the anchor-resultant dict + force residual +
        moment residual.

    T8 integration convention: the anchor_loads values are the forces the
    members apply to the chassis anchors (i.e. the loads[n] of f_ext in
    solve_compliance); no sign flip is needed — pass them in directly.
    """
    f_cp = np.array([q.fx, q.fy, q.fz], float)
    m_cp = np.array([q.mx, q.my, q.mz], float)
    r = np.asarray(cp_rel, float)
    m_hub = m_cp + np.cross(r, f_cp)          # 刚体等效：力矩平移至轮心 — rigid-body equivalent: translate the moment to the wheel center
    A = np.column_stack([lk.unit() for lk in links])
    m, resid = solve_linear(A, -f_cp)
    for i, lk in enumerate(links):
        lk.mag = float(m[i])
    out: dict[str, np.ndarray] = {}
    for lk in links:
        end = _chassis_end(lk, hub_point)
        key = lk.id if lk.id else f"{end[0]:.0f},{end[1]:.0f},{end[2]:.0f}"
        # 车身端反力 = -mag * unit（构件端在 hub 的反作用传递到车身）
        # Chassis-end reaction = -mag * unit (the member's reaction at the hub is transmitted to the chassis)
        out[key] = out.get(key, np.zeros(3)) - lk.mag * lk.unit()
    return CornerLoadsResult(anchor_loads=out, residual=float(resid),
                             moment_residual=float(np.linalg.norm(m_hub)))
