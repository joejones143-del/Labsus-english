"""K&C 两层迭代求解器（S1 引擎内核·分水岭）。

内层：衬套位移 δ 写入锚点（小角刚体变换）→ solve_pose（现有 least_squares TRF 机构投影）。
外层：δ 上力平衡 r(δ) = f_ext − bush_force(δ)（TRF + 数值雅可比 + Anderson 兜底）。
状态机：VALID / APPROXIMATE（最小范数超静定）/ COMPLIANCE_DIVERGED / SOLVER_FAILED。

符号约定（load 与 δ 的关系）：
    load: 作用于衬套锚点的外载荷向量（N，全局坐标3分量）。
    δ: 衬套 6DOF 位移 [dx, dy, dz, rx, ry, rz]（mm / rad）。
    平衡方程: r(δ) = f_ext − bush_force(δ) = 0
    bush_force(δ)[i] = k[i] * δ[i]（线性刚度，S1 阶段无耦合/预紧简化）。
    向上力 load_z = +500 → δz = +500/k_z（衬套正向位移，成员相对锚点上移）。
    向下力 load_z = −500 → δz = −500/k_z（衬套负向位移，成员相对锚点下移/压缩）。
    测试断言若需验证位移 magnitude，使用 abs(δ) 以避免符号歧义。

S1 载荷模型：单锚点直承外载（简化）。
T5 将对接 T2 静力层（接地点外载 → 二力杆 → 锚点合力）替换 load。

返回后 mech 处于最终一致姿态；result.anchors 为最终锚点位置。

K&C two-layer iterative solver (S1 engine core · watershed).

Inner layer: bushing displacement δ written into anchors (small-angle
rigid-body transform) → solve_pose (the existing least_squares TRF
mechanism projection).
Outer layer: force balance over δ, r(δ) = f_ext − bush_force(δ)
(TRF + numeric Jacobian + Anderson fallback).
State machine: VALID / APPROXIMATE (minimum-norm indeterminate) /
COMPLIANCE_DIVERGED / SOLVER_FAILED.

Sign conventions (the relation between load and δ):
    load: the external load vector applied at the bushing anchors
    (N, 3 components in global coordinates).
    δ: the bushing 6DOF displacement [dx, dy, dz, rx, ry, rz]
    (mm / rad).
    Equilibrium: r(δ) = f_ext − bush_force(δ) = 0
    bush_force(δ)[i] = k[i] * δ[i] (linear stiffness; S1 has no
    coupling/preload yet — simplification).
    Upward force load_z = +500 → δz = +500/k_z (positive bushing
    displacement; the member moves up relative to the anchor).
    Downward force load_z = −500 → δz = −500/k_z (negative bushing
    displacement; the member moves down / compresses relative to the
    anchor).
    If a test assertion needs to verify the displacement magnitude, use
    abs(δ) to avoid sign ambiguity.

S1 load model: a single anchor directly carrying the external load
(simplified). T5 will swap in the T2 static layer (contact-patch load →
two-force members → anchor resultants) to replace `load`.

On return, mech is in a final consistent posture; result.anchors holds
the final anchor positions.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np
from scipy.optimize import least_squares

from src.components.bushing import Bushing6DOF, bush_force, bush_force_jac
from src.solver.compliance_transform import apply_bushing_to_anchors
from src.solver.mechanism.solver import solve_pose

if TYPE_CHECKING:
    # 仅用于 solve_compliance_full 的 `case` 形参标注；运行时由函数内
    # 惰性 import 保持"compliance 不依赖 forces"的边界（ruff F821 曾在此报警：
    # `case: "QSLoad"` 引用了从未导入的名字，注解不可解析）。
    # Only used for the `case` parameter annotation of solve_compliance_full;
    # at runtime a lazy import inside the function preserves the boundary
    # that "compliance does not depend on forces" (ruff F821 once flagged
    # here: `case: "QSLoad"` referenced a name that was never imported, so
    # the annotation was unresolvable).
    from src.solver.forces import QSLoad

# ── 状态判定阈值（模块常量） — status-decision thresholds (module constants) ──
COST_VALID = 1e-6         # 外层代价函数低于此值 → VALID — the outer cost function below this → VALID
COST_DIVERGED = 1e-3      # 外层代价函数高于此值且未收敛 → COMPLIANCE_DIVERGED — outer cost above this and not converged → COMPLIANCE_DIVERGED
KIN_TOL = 1e-10           # 内层运动学求解器公差 — inner kinematic solver tolerance
KIN_MAX_NFEV = 200        # 内层最大函数评估次数 — inner max function evaluations
ANDERSON_BETA = 0.5       # Anderson 混合系数（简单不动点步长） — Anderson blending coefficient (simple fixed-point step size)
ANDERSON_MEM = 3          # Anderson 历史深度 — Anderson history depth
ANDERSON_MAX_ITER = 40    # Anderson 最大迭代 — Anderson max iterations
ANDERSON_TOL = 1e-6       # Anderson 收敛阈值 — Anderson convergence threshold
ANDERSON_MAX_STEP = 5.0   # Anderson 单步最大位移（mm） — Anderson max per-step displacement (mm)


@dataclass
class ComplianceSolverResult:
    """K&C 求解结果。

    返回后 mech 处于最终一致姿态；anchors 为最终锚点位置。

    The K&C solve result.

    On return, mech is in a final consistent posture; `anchors` holds the
    final anchor positions.
    """
    status: str                               # VALID | APPROXIMATE | COMPLIANCE_DIVERGED | SOLVER_FAILED
    delta: dict[str, np.ndarray] = field(default_factory=dict)   # {衬套名: 6-DOF 位移} — {bushing name: 6-DOF displacement}
    kin_residual: float = 0.0                 # 内层运动学最大残差（mm） — inner-layer kinematic max residual (mm)
    force_balance_residual: float = 0.0       # 外层力平衡残差范数（N） — outer-layer force-balance residual norm (N)
    iterations: int = 0                       # 外层函数评估次数 — outer-layer function evaluations
    ms: float = 0.0                           # 总耗时（ms） — total wall time (ms)
    anchors: dict[str, np.ndarray] = field(default_factory=dict)  # 变换后的锚点位置 — transformed anchor positions
    bush_loads: dict[str, np.ndarray] = field(default_factory=dict)  # {衬套名: 6-抗力} — {bushing name: 6-component reaction}


# ── Anderson 加速不动点迭代（C1: TRF 失败后的兜底） ──
def _anderson(residual_fun, x0, *, beta: float = ANDERSON_BETA,
              m: int = ANDERSON_MEM, max_iter: int = ANDERSON_MAX_ITER,
              tol: float = ANDERSON_TOL) -> tuple[np.ndarray, float, bool]:
    """Anderson-M 加速不动点迭代 δ_{k+1} = δ_k + r(δ_k)。

    无雅可比，仅使用历史差商做最小二乘外推。
    返回 (x, residual_norm, converged)。

    Anderson-accelerated fixed-point iteration δ_{k+1} = δ_k + r(δ_k).

    No Jacobian; only historical difference quotients are used for the
    least-squares extrapolation.
    Returns (x, residual_norm, converged).
    """
    x = np.asarray(x0, float).copy()
    xs: list[np.ndarray] = []
    fs: list[np.ndarray] = []

    for _ in range(max_iter):
        r = np.asarray(residual_fun(x), float)
        rnorm = float(np.linalg.norm(r))
        if rnorm < tol:
            return x, rnorm, True

        xs.append(x.copy())
        fs.append(r.copy())
        if len(xs) > m:
            xs.pop(0)
            fs.pop(0)

        if len(xs) > 1:
            # 最小化 ||F(x)+F'(x)dx|| 的最小二乘外推（无雅可比，使用历史差商）
            # Least-squares extrapolation minimizing ||F(x)+F'(x)dx|| (no Jacobian; historical difference quotients)
            DX = np.column_stack([xs[k] - xs[k - 1] for k in range(1, len(xs))])
            DF = np.column_stack([fs[k] - fs[k - 1] for k in range(1, len(fs))])
            g, *_ = np.linalg.lstsq(DF, -fs[-1], rcond=None)
            x_new = xs[-1] + DX @ g
        else:
            x_new = x + beta * r

        # 限制步长
        # Limit the step size
        step = np.linalg.norm(x_new - xs[-1])
        if step > ANDERSON_MAX_STEP:
            x_new = xs[-1] + (x_new - xs[-1]) * (ANDERSON_MAX_STEP / step)
        x = x_new

    # 最终评估
    # Final evaluation
    r = np.asarray(residual_fun(x), float)
    rnorm = float(np.linalg.norm(r))
    return x, rnorm, rnorm < tol


def solve_compliance(
    mech,
    *,
    bushings: dict[str, Bushing6DOF],
    load: np.ndarray,
    loads: dict[str, np.ndarray] | None = None,
    travel: float,
    rack: float,
    applied_at: dict[str, str] | None = None,
    max_disp_step: float = 0.5,
    max_nfev: int = 60,
) -> ComplianceSolverResult:
    """K&C 单点求解。

    Parameters
    ----------
    mech : Mechanism
        机构模型（由 build_mechanism 构造）。
    bushings : dict
        {衬套名: Bushing6DOF} 衬套集合。空字典 → 纯运动学回退。
    load : np.ndarray, shape (3,)
        默认外载荷向量（N，全局坐标），广播到所有衬套。
    loads : dict, optional
        {衬套名: np.ndarray(3,)} 各衬套独立外载荷。提供时覆盖 load 广播。
    travel, rack : float
        轮跳 / 齿条位移（mm），传入内层 solve_pose。
    applied_at : dict, optional
        {衬套名: 节点名} 载荷作用点映射（保留参数，T5 扩展用）。
    max_disp_step : float
        外层 TRF 变量缩放（mm），控制最大步长。
    max_nfev : int
        外层最大函数评估次数。

    Returns
    -------
    ComplianceSolverResult
        返回后 mech 处于最终一致姿态；anchors 为最终锚点位置。

    Single-point K&C solve.

    Parameters
    ----------
    mech : Mechanism
        The mechanism model (built by build_mechanism).
    bushings : dict
        {bushing name: Bushing6DOF} bushing set. An empty dict → pure
        kinematics fallback.
    load : np.ndarray, shape (3,)
        Default external load vector (N, global coordinates), broadcast
        to all bushings.
    loads : dict, optional
        {bushing name: np.ndarray(3,)} per-bushing external loads. When
        given, overrides the broadcast of `load`.
    travel, rack : float
        Wheel travel / rack displacement (mm), passed to the inner
        solve_pose.
    applied_at : dict, optional
        {bushing name: node name} load application-point mapping
        (reserved parameter, for the T5 extension).
    max_disp_step : float
        Outer TRF variable scaling (mm); controls the maximum step.
    max_nfev : int
        Outer maximum function evaluations.

    Returns
    -------
    ComplianceSolverResult
        On return, mech is in a final consistent posture; `anchors` holds
        the final anchor positions.
    """
    t0 = time.perf_counter()
    names = list(bushings.keys())
    nd = len(names)

    # ── 无衬套回退：纯运动学 ──
    # No-bushing fallback: pure kinematics
    if nd == 0:
        rep = solve_pose(mech, travel, rack)
        return ComplianceSolverResult(
            status="VALID",
            kin_residual=rep.residual,
            force_balance_residual=0.0,
            iterations=0,
            ms=(time.perf_counter() - t0) * 1000.0,
        )

    # 载荷映射：per-bushing loads 优先，否则广播 load
    # Load mapping: per-bushing loads take priority, otherwise broadcast `load`
    load_broadcast = np.asarray(load, float)
    bush_ext: dict[str, np.ndarray] = {}
    for n in names:
        if loads is not None and n in loads:
            bush_ext[n] = np.asarray(loads[n], float)[:3].copy()
        else:
            bush_ext[n] = load_broadcast[:3].copy()

    def _unpack(x: np.ndarray) -> dict[str, np.ndarray]:
        return {n: x[6 * k: 6 * k + 6] for k, n in enumerate(names)}

    # ── 外层残差：力平衡 r(δ) = f_ext − bush_force(δ) ──
    # kin_last 通过结果赋值回传（M2: 避免 mutable closure）
    # kin_last is returned by assignment through the result (M2: avoids a mutable closure)
    _kin_last = 0.0

    def _residual(x: np.ndarray) -> np.ndarray:
        nonlocal _kin_last
        d = _unpack(x)
        # 1) 内层：衬套位移 → 锚点变换 → 机构重解
        # 1) Inner: bushing displacement → anchor transform → re-solve the mechanism
        anchors = apply_bushing_to_anchors(bushings, d)
        for node, pos in anchors.items():
            mech.nodes[node].pos = pos.copy()
        rep = solve_pose(mech, travel, rack, max_nfev=KIN_MAX_NFEV, tol=KIN_TOL)
        _kin_last = rep.residual

        # 2) 外层：力平衡残差
        # S1: 外载仅作用于平移 DOF（3分量）；旋转 DOF 无外部力矩（M5 注释），
        # 因为 K&C 测试中衬套仅传递平移力，旋转约束由机构铰链承担。
        # S1: the external load acts only on the translational DOFs (3
        # components); the rotational DOFs have no external torques (the M5
        # note), because in the K&C tests the bushings transmit only
        # translational forces and the rotational constraints are carried
        # by the mechanism hinges.
        r = np.zeros(6 * nd)
        for k, n in enumerate(names):
            f_ext = np.zeros(6)
            f_ext[:3] = bush_ext[n]   # S1: 仅平移外载；旋转 DOF 无外部力矩 — S1: translational external load only; rotational DOFs have no external torque
            r[6 * k: 6 * k + 6] = f_ext - bush_force(bushings[n], d[n])
        return r

    # F-65（2026-08-30）：解析雅可比接线 —— bush_force_jac 此前完整实现却
    # 无人调用，外层 TRF 全程数值差分（每迭代 (6·nd+1) 次残差评估 × 每次
    # 内层 200-nfev 机构重解，多衬套时 O(nd) 恶化）。解析雅可比为块对角：
    #   ∂r_k/∂δ_k = -J_bush,k（衬套独立；耦合矩阵在 bush_force_jac 内生效）
    # F-65 (2026-08-30): analytic Jacobian wiring — bush_force_jac was fully
    # implemented but never called, so the outer TRF did numeric
    # differencing the whole way (per iteration (6·nd+1) residual
    # evaluations × each requiring a 200-nfev mechanism re-solve,
    # degrading as O(nd) with multiple bushings). The analytic Jacobian is
    # block-diagonal:
    #   ∂r_k/∂δ_k = -J_bush,k (bushings independent; the coupling matrix
    #   takes effect inside bush_force_jac)
    def _residual_jac(x: np.ndarray) -> np.ndarray:
        d = _unpack(x)
        J = np.zeros((6 * nd, 6 * nd))
        for k, n in enumerate(names):
            J[6 * k: 6 * k + 6, 6 * k: 6 * k + 6] = -bush_force_jac(bushings[n], d[n])
        return J

    # ── 外层求解：scipy TRF（解析雅可比）──
    # Outer solve: scipy TRF (analytic Jacobian)
    x0 = np.zeros(6 * nd)
    trf_nfev = 0
    try:
        res = least_squares(
            _residual, x0, method="trf", max_nfev=max_nfev,
            xtol=1e-8, ftol=1e-8, gtol=1e-8,
            x_scale=np.ones(6 * nd) * max_disp_step,
            jac=_residual_jac,
        )
        trf_nfev = int(res.nfev)
    except Exception:
        # TRF 异常 → 尝试 Anderson 兜底
        # TRF raised → try the Anderson fallback
        res = None

    # ── C1: Anderson 固定点回退 ──
    # C1: Anderson fixed-point fallback
    anderson_ok = False
    anderson_rnorm = float("inf")
    if res is None or (not res.success and res.cost > COST_DIVERGED):
        anderson_x, anderson_rnorm, anderson_ok = _anderson(_residual, x0)
        anderson_nfev = ANDERSON_MAX_ITER  # 估算 — estimate
        if res is None:
            # TRF 完全失败 → 用 Anderson 结果
            # TRF failed completely → use the Anderson result
            res_x = anderson_x
            final_nfev = anderson_nfev
        else:
            # TRF 和 Anderson 都跑了 → 选更好的
            # Both TRF and Anderson ran → pick the better one
            trf_rnorm = float(np.sqrt(2 * res.cost))
            if anderson_rnorm < trf_rnorm:
                res_x = anderson_x
                final_nfev = trf_nfev + anderson_nfev
            else:
                res_x = res.x
                final_nfev = trf_nfev
    else:
        res_x = res.x
        final_nfev = trf_nfev

    d = _unpack(res_x)

    # ── 状态判定 ──
    # Status decision
    # 重新评估最终残差以获取精确代价
    # Re-evaluate the final residual to obtain the exact cost
    final_r = np.asarray(_residual(res_x), float)
    final_rnorm = float(np.linalg.norm(final_r))
    final_cost = 0.5 * final_rnorm ** 2

    if final_cost < COST_VALID:
        status = "VALID"
    elif anderson_ok:
        # Anderson 收敛（即使 TRF 没收敛）→ VALID
        # Anderson converged (even though TRF did not) → VALID
        status = "VALID"
    elif final_cost > COST_DIVERGED:
        status = "COMPLIANCE_DIVERGED"
    else:
        status = "APPROXIMATE"

    # ── C2: 最终一致性写回 ──
    # 将最优 δ 的锚点位置写入 mech.nodes，并运行最终 solve_pose 确保机构一致
    # Write the optimal δ's anchor positions into mech.nodes and run a final
    # solve_pose to ensure the mechanism is consistent
    final_anchors = apply_bushing_to_anchors(bushings, d)
    for node, pos in final_anchors.items():
        mech.nodes[node].pos = pos.copy()
    solve_pose(mech, travel, rack, max_nfev=KIN_MAX_NFEV, tol=KIN_TOL)

    return ComplianceSolverResult(
        status=status,
        delta=d,
        kin_residual=_kin_last,
        force_balance_residual=final_rnorm,
        iterations=final_nfev,
        ms=(time.perf_counter() - t0) * 1000.0,
        anchors=final_anchors,
        bush_loads={n: bush_force(bushings[n], d[n]) for n in names},
    )


# ── T8: K&C 全链路求解 ──────────────────────────────────────────────
# T8: full-chain K&C solve

def solve_compliance_full(
    mech,
    *,
    bushings: dict[str, Bushing6DOF],
    case: QSLoad,
    travel: float,
    rack: float,
    mk_links=None,
    cp_rel=None,
    tire_radius: float | None = None,
) -> ComplianceSolverResult:
    """K&C 全链路：接地点外载 →（二力杆静力）→ 锚点合力 → solve_compliance。

    mk_links: (mech) -> list[LinkForce] 回调；默认构造 PUSH(UP4→CH5) +
              LCA 前/后杆(CH3/CH4→UP2) + UCA 前/后杆(CH1/CH2→UP1)。
    cp_rel: 接地点相对轮心（默认 [0,0,-tire_radius]；tire_radius 缺省 325，
            与 API v3models.CaseLoad/KandcRequest.tire_radius 默认一致——
            F-64：旧默认 [0,0,-320] 与 API 默认 325 自相矛盾）。
    tire_radius: 轮胎半径 mm，仅用于派生默认 cp_rel。

    F-64（2026-08-30）：状态机兑现 docstring 承诺——
    - 超静定力残差（corner.residual）非零 → APPROXIMATE（根因：最小范数解
      是近似仲裁，非精确刚度分配）；
    - 外力矩折算到轮心后的残余 moment_residual 非零 → APPROXIMATE（球铰
      模型无法平衡力矩），并在返回值上携带说明（warnings 合并入 status）。

    Full-chain K&C: contact-patch external load → (two-force-member
    statics) → anchor resultants → solve_compliance.

    mk_links: a (mech) -> list[LinkForce] callback; by default builds
              PUSH (UP4→CH5) + LCA front/rear links (CH3/CH4→UP2) + UCA
              front/rear links (CH1/CH2→UP1).
    cp_rel: contact patch relative to the wheel center (default
            [0,0,-tire_radius]; tire_radius defaults to 325, consistent
            with the API v3models.CaseLoad/KandcRequest.tire_radius
            default — F-64: the old default [0,0,-320] contradicted the
            API default 325).
    tire_radius: tire radius in mm; only used to derive the default
            cp_rel.

    F-64 (2026-08-30): the state machine now honors the docstring's
    promise —
    - a non-zero statically-indeterminate force residual
      (corner.residual) → APPROXIMATE (root cause: the minimum-norm
      solution is an approximate arbitration, not an exact stiffness
      allocation);
    - a non-zero moment_residual after translating the external moments
      to the wheel center → APPROXIMATE (the ball-joint model cannot
      balance moments), and the return value carries an explanation
      (warnings merged into the status).
    """
    from src.solver.forces import LinkForce, _chassis_end, corner_to_anchor_loads

    # 1) 运动学求解（确认机构可达当前姿态）
    # 1) Kinematic solve (confirm the mechanism can reach the current posture)
    rep = solve_pose(mech, travel, rack)
    if not rep.ok:
        return ComplianceSolverResult(status="SOLVER_FAILED", kin_residual=rep.residual)

    # 2) 构造二力杆列表
    # 2) Build the two-force-member list
    if mk_links is not None:
        links = mk_links(mech)
    else:
        links = [
            LinkForce(a=mech.node("UP4").pos.copy(), b=mech.node("CH5").pos.copy(), id="PUSH"),
            LinkForce(a=mech.node("CH3").pos.copy(), b=mech.node("UP2").pos.copy(), id="LCA_F"),
            LinkForce(a=mech.node("CH4").pos.copy(), b=mech.node("UP2").pos.copy(), id="LCA_R"),
            LinkForce(a=mech.node("CH1").pos.copy(), b=mech.node("UP1").pos.copy(), id="UCA_F"),
            LinkForce(a=mech.node("CH2").pos.copy(), b=mech.node("UP1").pos.copy(), id="UCA_R"),
        ]

    # 3) 接地点外载 → 各杆轴向力 → 车身锚点合力
    # 3) Contact-patch load → member axial forces → chassis anchor resultants
    hub_point = mech.node(mech.wheel).pos.copy()
    if cp_rel is None:
        cp_rel = np.array([0.0, 0.0, -(tire_radius if tire_radius is not None else 325.0)])
    corner = corner_to_anchor_loads(case, links, hub_point=hub_point, cp_rel=cp_rel)

    # 4) 杆车身端 → 节点名（F-63 修复，2026-08-30：atol 由 1.0 收紧到 1e-6 并
    #    做唯一性校验；匹配失败抛异常——宁可显式 422/500，不可静默零载荷。
    #    旧实现 allclose(atol=1.0) 失败时返回 "" → per_node 缺键 → 衬套载荷 0，
    #    compliance toe 恒 0 而表面"正常"）
    # 4) Member chassis end → node name (F-63 fix, 2026-08-30: atol
    #    tightened from 1.0 to 1e-6 with a uniqueness check; a failed match
    #    raises — prefer an explicit 422/500 over silently zero loads.
    #    The old implementation returned "" when allclose(atol=1.0) failed
    #    → per_node missing keys → bushing loads 0 → compliance toe stuck
    #    at 0 while looking "normal")
    def _node_name(pos: np.ndarray) -> str:
        hits = [nm for nm, nd in mech.nodes.items()
                if float(np.linalg.norm(np.asarray(nd.pos, float) - pos)) < 1e-6]
        if len(hits) > 1:
            raise ValueError(
                f"compliance: anchor position coincides with multiple nodes {hits} (spacing <1e-6mm); cannot map uniquely")
        if not hits:
            raise ValueError(
                f"compliance: member chassis-end anchor {np.round(np.asarray(pos, float), 2).tolist()} "
                "does not map to any mechanism node (positions do not coincide) — load mapping failures must raise explicitly")
        return hits[0]

    # 5) 按节点聚合力（anchor_loads 值已是车身端力，直接累加）
    # 5) Aggregate forces per node (the anchor_loads values are already chassis-end forces; accumulate directly)
    per_node: dict[str, np.ndarray] = {}
    for lk in links:
        end = _chassis_end(lk, hub_point)
        nm = _node_name(end)
        per_node[nm] = per_node.get(nm, np.zeros(3)) + corner.anchor_loads.get(lk.id, np.zeros(3))

    # 6) 衬套载荷映射：{衬套名: 对应成员节点合力}
    # 6) Bushing load mapping: {bushing name: the resultant at the corresponding member node}
    loads: dict[str, np.ndarray] = {}
    for name, b in bushings.items():
        node = next(iter(b.member_nodes))
        loads[name] = per_node.get(node, np.zeros(3))

    # 7) 调用 T4 求解器
    # 7) Call the T4 solver
    res = solve_compliance(mech, bushings=bushings, loads=loads,
                           load=np.zeros(3), travel=travel, rack=rack)

    # F-64（2026-08-30）：状态机兑现 docstring 承诺——超静定最小范数残差与
    # 力矩折算残余无法由二力杆/球铰模型精确闭合，条件满足时降级 APPROXIMATE。
    # F-64 (2026-08-30): the state machine honors the docstring promise — the
    # indeterminate minimum-norm residual and the translated-moment residual
    # cannot be closed exactly by the two-force-member/ball-joint model;
    # downgrade to APPROXIMATE when the conditions are met.
    if res.status in ("VALID", "APPROXIMATE"):
        if corner.residual > 1e-6:
            res.status = "APPROXIMATE"
        if corner.moment_residual > 1e-6:
            res.status = "APPROXIMATE"
    return res
