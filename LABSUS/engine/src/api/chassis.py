"""/api/v3 整车分析业务层（S2-5；quasi 内核 P0 统一 2026-08-22）。

四角装配（FR/FL/RR/RL）：
- FR = vehicle.front.points（右轮，X=外侧）；FL = 镜像（x→−x）+ steer_axis 反号；
- RR = vehicle.rear.points；RL = 镜像。
每角用引擎机制求解器（solve_pose）出定位角；准静态载荷转移与前端
solveQuasiStatic 同一内核（侧倾耦合迭代 3 轮：rc_h 引擎扫掠迁移 +
kw 前端曲线迁移 → 三路径解耦 + TLLTD 随 gy 单调变化）。

坐标系与前端一致：X=外侧 / Y=向前 / Z=向上；轮跳正 = 压缩。

English: full-vehicle analysis business layer (S2-5; quasi kernel unified P0,
2026-08-22). Four-corner assembly (FR/FL/RR/RL): FR = vehicle.front.points (right
wheel, X outboard); FL = mirrored (x->-x) + steer_axis sign-flipped; RR =
vehicle.rear.points; RL = mirrored. Each corner's alignment angles come from the
engine mechanism solver (solve_pose); the quasi-static load transfer shares one kernel
with the frontend solveQuasiStatic (3-round roll-coupled iteration: rc_h engine-sweep
migration + kw frontend-curve migration -> three paths decoupled + TLLTD monotonically
varying with gy). Coordinate system matches the frontend: X outboard / Y forward /
Z up; positive wheel travel = compression.
"""
from __future__ import annotations

import bisect
import math
import threading
import time
from collections import OrderedDict

import numpy as np

from src.api.v3models import (
    AxleSpec,
    ChassisLoads,
    ChassisPoseResponse,
    ChassisRequest,
    ChassisSweepResponse,
    DesignSpec,
    QuasiInputs,
    VehicleSpec,
)
from src.api.v3service import _new_mech, mirror_left, pose_metrics
from src.metrics.kandc import _slope  # F-13：单一差分内核（全局共用） — single differencing kernel (shared globally)
from src.solver.mechanism.solver import solve_pose
from src.tire_mf import cornering_stiffness  # M5：Cα 唯一实现（避免第三份拷贝） — M5: the single Cα implementation (avoids a third copy)

R2D = 180.0 / math.pi
D2R = math.pi / 180.0
G = 9.81

CORNER_KEYS = ("FL", "FR", "RL", "RR")


def make_corners(vehicle: VehicleSpec) -> dict[str, dict]:
    """四角装配：返回 {角名: {points, left: bool}}。

    English: four-corner assembly — returns {corner name: {points, left: bool}}.
    """
    f_r = vehicle.front.points
    r_r = vehicle.rear.points
    return {
        "FR": {"points": f_r, "left": False},
        "FL": {"points": mirror_left(f_r), "left": True},
        "RR": {"points": r_r, "left": False},
        "RL": {"points": mirror_left(r_r), "left": True},
    }


def corner_pose(ax: AxleSpec, points: dict, travel: float, rack: float,
                left: bool) -> tuple[dict, float, list[str]]:
    """单角求解：返回 (metrics, residual, warnings)。P2：arch→strut 附着拓扑。

    English: single-corner solve — returns (metrics, residual, warnings). P2:
    arch->strut attach topology.
    """
    mech = _new_mech(points, left=left, arch=ax.arch)
    rep = solve_pose(mech, travel, rack)
    m = pose_metrics(mech, ax.tire_radius,
                     DesignSpec(camber_deg=ax.camber_deg, toe_deg=ax.toe_deg))
    warn = [] if rep.ok else [f"{'L' if left else 'R'} corner residual {rep.residual:.3f} mm"]
    m["residual"] = float(rep.residual)
    return m, float(rep.residual), warn


def mr_at_zero(ax: AxleSpec, points: dict, left: bool) -> float:
    """MR@0：damper 对轮跳的中心差分导数绝对值（|ΔL/Δtravel|，±2mm）。

    P2（2026-08-22）：依赖 rocker 真实三维轴 + strut_attach（ax.arch）；
    旧固定 X 轴下压缩侧摇臂无解 → RK_DAMPER 不更新 → ±2mm 差分失真（3.98）。

    MR 取绝对值：下游仅用 mr²（kw=kS·mr²、kphi∝mr²），符号无关。符号在
    pullrod/uca 布局可为负（压缩行程减振器伸长，实测后轴 -0.167），若不加
    abs 会被旧 max(0.05) 地板掩盖。分支/数值与前端 mrRef 常量的差距
    （前 0.47 vs 0.75、后 0.17 vs 0.78）登记 OpenItem-B：摇臂双根分支选择
    （最近根 vs 投影全局解）与 FE 弹簧力平衡分支的差异。

    F-67（2026-08-30）：不再吞异常 —— 任一姿态求解失败即抛出让调用方
    _resolve_mr 统一兜底（此前 except 静默返回 0.75，后轴也被回退到前轴
    常量）；同时检查 solve_pose 的 rep.ok，失败解的 damper 长度不再进 MR。

    English: MR@0 — absolute value of the central-difference derivative of damper length
    vs wheel travel (|ΔL/Δtravel|, ±2mm). P2 (2026-08-22): depends on the rocker real 3D
    axis + strut_attach (ax.arch); with the old fixed X axis the rocker compression side
    had no solution -> RK_DAMPER not updated -> ±2mm differencing distorted (3.98).
    MR takes the absolute value: downstream only uses mr² (kw=kS·mr², kphi∝mr²),
    sign-independent. The sign can be negative in pullrod/uca layouts (the damper
    extends under compression; measured rear -0.167); without abs() it would be masked
    by the old max(0.05) floor. The branch/value gap vs the frontend mrRef constants
    (front 0.47 vs 0.75, rear 0.17 vs 0.78) is logged as OpenItem-B: the rocker dual-root
    branch selection (nearest root vs projected global solution) and the FE spring-force
    balance branch difference.
    F-67 (2026-08-30): no longer swallows exceptions — any pose-solve failure raises so
    the caller _resolve_mr handles fallback uniformly (previously except silently
    returned 0.75, so the rear axle also fell back to the front constant); also checks
    solve_pose's rep.ok — failed solutions' damper lengths no longer enter MR.
    """
    mech = _new_mech(points, left=left, arch=ax.arch)
    rep1 = solve_pose(mech, 2.0, 0.0)
    if not rep1.ok:
        raise RuntimeError(f"mr_at_zero: +2mm pose residual {rep1.residual:.3f}mm > 0.02")
    d1 = mech.node("RK_DAMPER").pos - mech.node("DAMPER_CHASSIS").pos
    lp = float(np.linalg.norm(d1))
    mech2 = _new_mech(points, left=left, arch=ax.arch)
    rep2 = solve_pose(mech2, -2.0, 0.0)
    if not rep2.ok:
        raise RuntimeError(f"mr_at_zero: -2mm pose residual {rep2.residual:.3f}mm > 0.02")
    d2 = mech2.node("RK_DAMPER").pos - mech2.node("DAMPER_CHASSIS").pos
    lm = float(np.linalg.norm(d2))
    return max(0.02, abs(-(lp - lm) / 4.0))


def arb_geom(points: dict, arb) -> dict:
    """横向稳定杆几何（镜像前端 arbGeom，全部用设计位硬点）。

    English: anti-roll bar geometry (mirrors frontend arbGeom; all points at the design
    position).
    """
    F = np.asarray(points["LCA_F"], float)
    B = np.asarray(points["LBJ"], float)
    t = arb.t
    P0 = F + (B - F) * t
    xa = float(P0[0])
    ay = float(F[1] + arb.dy)
    az = float(F[2] + arb.dz)
    a = ay - float(P0[1])
    E0 = np.array([xa, ay - a, az])
    ldl = float(np.linalg.norm(E0 - P0))
    P = F + (B - F) * t
    A = ay - float(P[1])
    Bz = az - float(P[2])
    R = math.hypot(A, Bz)
    K = (xa - float(P[0])) ** 2 + A * A + Bz * Bz + a * a - ldl * ldl
    denom = 2 * a * R
    c = float(np.clip(K / (denom if abs(denom) > 1e-9 else 1e-9), -1.0, 1.0))
    f0 = math.atan2(Bz, A)
    sg = 1.0 if math.atan2(az - float(P0[2]), ay - float(P0[1])) >= 0 else -1.0
    psi = f0 - sg * math.acos(c)
    E = np.array([xa, ay - a * math.cos(psi), az - a * math.sin(psi)])
    return {"xa": xa, "ay": ay, "az": az, "a": a, "ldl": ldl, "P": P,
            "P0": P0, "E": E, "psi": psi}


def arb_rate(ax: AxleSpec) -> dict:
    """稳定杆扭转刚度 → 等效侧倾刚度系数（镜像前端 arbRate，mrArb=0.55 常量）。

    English: ARB torsional stiffness -> equivalent roll-stiffness coefficient (mirrors
    frontend arbRate; mrArb=0.55 constant).
    """
    d = ax.arb.d
    if d <= 0.5:
        return {"k": 0.0, "J": 0.0, "kt": 0.0}
    J = math.pi * d ** 4 / 32.0
    g = arb_geom(ax.points, ax.arb)
    L = 2 * g["xa"]
    kt = ax.arb.G * J / L
    mr_arb = 0.55
    k = kt / (g["a"] * g["a"]) * mr_arb * mr_arb
    return {"k": k, "J": J, "kt": kt, "a": g["a"], "L": L}


_RC_SWEEP_CACHE: OrderedDict[tuple, dict] = OrderedDict()
_RC_SWEEP_LOCK = threading.Lock()  # F-70：FastAPI 端点跑线程池，缓存读写加锁 — F-70: FastAPI endpoints run on a thread
                                  # pool; lock cache reads/writes


def axle_rc_sweep(ax: AxleSpec, lim: float = 90.0, n: int = 21) -> dict:
    """单轴 rc_h-行程扫掠（右轮机构，rack=0）——侧倾耦合迭代用。

    rc_h 只依赖双叉臂几何（CH1-4 / LBJ / UBJ / WC），不受 STRUT_OUT 拓扑
    影响，引擎可权威计算（对比：damper 链 MR 在引擎拓扑下失真，kw 迁移
    改由前端 kw_curve 下发）。按轴参数缓存，重复 solve 免重算。

    F-70（2026-08-30）：dict 淘汰/写入此前无锁（CPython 下竞态良性但
    非零风险）；加锁保证并发的 Hit/Miss 与 LRU 淘汰原子化。
    M5（2026-09-20）：docstring 一直自称 LRU，实现却是 FIFO（`pop(next(iter))`
    淘汰最早插入项）——改为 OrderedDict + move_to_end / popitem(last=False)，
    实现与声明一致。

    English: single-axle rc_h vs travel sweep (right-corner mechanism, rack=0) — used by
    the roll-coupled iteration. rc_h depends only on the double-wishbone geometry
    (CH1-4 / LBJ / UBJ / WC), independent of STRUT_OUT topology, so the engine computes
    it authoritatively (by contrast the damper-chain MR is distorted under the engine
    topology, so kw migration is delivered by the frontend kw_curve). Cached per axle
    parameters; repeated solves are free.
    F-70 (2026-08-30): dict eviction/writes were previously unlocked (benign race on
    CPython but non-zero risk); the lock makes concurrent hits/misses and LRU eviction
    atomic. M5 (2026-09-20): the docstring always claimed LRU while the implementation
    was FIFO (`pop(next(iter))`, evicting the earliest-inserted item) — now OrderedDict
    + move_to_end / popitem(last=False), implementation matching the claim.
    """
    key = (tuple(sorted((k, tuple(map(float, v))) for k, v in ax.points.items())),
           float(ax.tire_radius), float(ax.camber_deg), float(ax.toe_deg),
           ax.arch, float(lim), int(n))
    with _RC_SWEEP_LOCK:
        hit = _RC_SWEEP_CACHE.get(key)
        if hit is not None:
            _RC_SWEEP_CACHE.move_to_end(key)   # 真 LRU：命中即刷新新鲜度 — true LRU: refresh recency on hit
            return hit
    mech = _new_mech(ax.points, arch=ax.arch)
    design = DesignSpec(camber_deg=ax.camber_deg, toe_deg=ax.toe_deg)
    xs = [float(v) for v in np.linspace(-lim, lim, n)]
    rc: list[float | None] = []
    for t in xs:
        solve_pose(mech, t, 0.0)
        m = pose_metrics(mech, ax.tire_radius, design)
        rc.append(None if m["rc_h"] is None else float(m["rc_h"]))
    out = {"travel": xs, "rc_h": rc}
    with _RC_SWEEP_LOCK:
        while len(_RC_SWEEP_CACHE) >= 16:
            _RC_SWEEP_CACHE.popitem(last=False)   # 淘汰最久未使用项 — evict the least-recently-used item
        _RC_SWEEP_CACHE[key] = out
    return out


def _interp_fb(xs: list[float], ys: list, x: float, fb: float) -> float:
    """线性插值 + 端点钳位（镜像前端 sampleSweep）；任一邻点 None → 回退 fb。

    English: linear interpolation + endpoint clamping (mirrors frontend sampleSweep);
    either neighbor None -> fall back to fb.
    """
    if x <= xs[0]:
        return ys[0] if ys[0] is not None else fb
    if x >= xs[-1]:
        return ys[-1] if ys[-1] is not None else fb
    i = bisect.bisect_right(xs, x)
    y0, y1 = ys[i - 1], ys[i]
    if y0 is None or y1 is None:
        return fb
    t = (x - xs[i - 1]) / ((xs[i] - xs[i - 1]) or 1e-9)
    return float(y0 + (y1 - y0) * t)


def _kw_at(ax: AxleSpec, tr: float, kw0_nm: float) -> float:
    """轮端刚度 @行程（N/m）：kw_curve 插值（N/mm→N/m）优先，否则常数 kw0。

    English: wheel-corner stiffness at travel (N/m): kw_curve interpolation (N/mm->N/m)
    takes priority, otherwise the constant kw0.
    """
    cur = ax.kw_curve
    if cur is None:
        return kw0_nm
    v = _interp_fb(cur.travel, cur.kw, tr, math.nan)
    if v is None or not math.isfinite(v) or v <= 0:
        return kw0_nm
    return v * 1000.0


def quasi_loads(vehicle: VehicleSpec, q: QuasiInputs,
                mr: dict[str, float], rcH: dict[str, float],
                rc_sw: dict[str, dict],
                warnings: list[str] | None = None,
                tire=None) -> ChassisLoads:
    """准静态载荷转移 —— 侧倾耦合迭代（镜像前端 solveQuasiStatic 修复版）。

    3 轮迭代：每轮由当前侧倾角解外侧（压缩侧）轮行程差 dt=roll·半轮距 →
    rc_h 取引擎扫掠在 dt 处值（GEO 项随行程迁移）、kw 取 ±dt 处均值
    （ELA 项迁移，kw_curve 优先否则常数）→ 更新前后侧倾刚度 → 重解侧倾角。
    迁移后三路径不再与 ay 线性齐次 → TLLTD 随 gy 真实单调变化。

    warnings：可选收集列表（F-16：侧倾失稳时显式上报而非静默置零）。

    English: quasi-static load transfer — roll-coupled iteration (mirrors the fixed
    frontend solveQuasiStatic). 3 rounds: each round derives the outer (compression-side)
    wheel-travel difference dt = roll·half-track from the current roll angle -> rc_h from
    the engine sweep at dt (GEO term migrates with travel) and kw as the ±dt mean (ELA
    term migrates; kw_curve preferred, else constant) -> update front/rear roll stiffness
    -> re-solve the roll angle. After migration the three paths are no longer linearly
    homogeneous in ay -> TLLTD genuinely varies monotonically with gy.
    warnings: optional collection list (F-16: report roll instability explicitly instead
    of silently zeroing).
    """
    wb = vehicle.wheelbase_mm
    L = wb / 1000.0
    mS = vehicle.sprung_mass_kg
    mT = vehicle.mass_kg
    mU_f = vehicle.front.unsprung_kg * 2
    mU_r = vehicle.rear.unsprung_kg * 2

    f_ms = vehicle.front.spring_mass_kg
    r_ms = vehicle.rear.spring_mass_kg
    # ── a/b 口径（M5 2026-09-20 修复前后轴对调）────────────────────────────
    # a = CG→前轴距、b = CG→后轴距（讲义 ch10 §2.1：z_ra = (b/L)·z_rc,f + (a/L)·z_rc,r
    # 即前轴权重 = b/L）。两轴簧载按点质量模型：a = L·m_r/(m_f+m_r)、
    # b = L·m_f/(m_f+m_r) —— 簧载重的一端离它更近、且承担更多静载。
    # 旧实现写成 b = L·m_r/(m_f+m_r)（把"后簧载份额"当成 CG→后轴距），
    # 使前后轴静载整体对调：实测 f_ms=300/r_ms=340 时前轴占比 52.2%，
    # 而物理正确值应为 46.9%（簧载 300/640）。见 tests/test_static_split_invariant.py。
    # English: a/b accounting (M5 2026-09-20 front/rear swap fix): a = CG->front-axle
    # distance, b = CG->rear-axle distance (lecture ch10 §2.1: z_ra = (b/L)·z_rc,f +
    # (a/L)·z_rc,r, i.e. front-axle weight = b/L). The two axles' sprung masses follow a
    # point-mass model: a = L·m_r/(m_f+m_r), b = L·m_f/(m_f+m_r) — the heavier sprung end
    # sits nearer its axle and carries more static load. The old code wrote b =
    # L·m_r/(m_f+m_r) (treating the rear sprung share as the CG->rear-axle distance),
    # swapping the front/rear static loads: with f_ms=300 / r_ms=340 the front share
    # measured 52.2% but the physically correct value is 46.9% (sprung 300/640). See
    # tests/test_static_split_invariant.py.
    b = L * (f_ms / (f_ms + r_ms))
    a = L - b

    tf = (abs(vehicle.front.points["WC"][0]) * 2) / 1000.0
    tr = (abs(vehicle.rear.points["WC"][0]) * 2) / 1000.0
    huf = vehicle.front.tire_radius / 1000.0
    hur = vehicle.rear.tire_radius / 1000.0
    hs = vehicle.hs_mm / 1000.0

    ay = q.gy * G
    ax = q.gx * G

    # 初值/回退（F-15，2026-08-30）：`or 45.0` 会吞掉合法 0 值（侧倾中心恰在
    # 地面）；改为仅对 None/NaN/非正回退。MR≤0 物理退化（摇臂失效）仍走回退。 — initial
    # values / fallback (F-15, 2026-08-30): `or 45.0` swallowed a legitimate 0 (roll
    # center exactly at the ground); now falls back only for None/NaN/non-positive.
    # MR≤0 physical degeneration (rocker failure) still falls back.
    _zf = rcH.get("FR")
    _zr = rcH.get("RR")
    zrc_f0 = _zf if (_zf is not None and math.isfinite(_zf)) else 45.0
    zrc_r0 = _zr if (_zr is not None and math.isfinite(_zr)) else 65.0
    zrc_f, zrc_r = zrc_f0 / 1000.0, zrc_r0 / 1000.0

    _mf = mr.get("front")
    _mrr = mr.get("rear")
    mrF = _mf if (_mf is not None and math.isfinite(_mf) and _mf > 0.05) else 0.75
    mrR = _mrr if (_mrr is not None and math.isfinite(_mrr) and _mrr > 0.05) else 0.78
    kw_f0 = vehicle.front.spring_rate * 1000.0 * mrF * mrF   # N/m
    kw_r0 = vehicle.rear.spring_rate * 1000.0 * mrR * mrR

    arF = arb_rate(vehicle.front)
    arR = arb_rate(vehicle.rear)
    kphi_arb_f = arF["k"] * 1000.0 * tf * tf / 2.0
    kphi_arb_r = arR["k"] * 1000.0 * tr * tr / 2.0

    kphi_f = 0.5 * kw_f0 * tf * tf + kphi_arb_f
    kphi_r = 0.5 * kw_r0 * tr * tr + kphi_arb_r
    kphi_tot = kphi_f + kphi_r
    roll_rad = 0.0

    swf, swr = rc_sw["front"], rc_sw["rear"]
    half_f = abs(vehicle.front.points["WC"][0])   # mm
    half_r = abs(vehicle.rear.points["WC"][0])
    _roll_unstable = False
    for _ in range(3):
        hra = zrc_f + (a / L) * (zrc_r - zrc_f)
        h_arm = max(0.05, hs - hra)
        denom = kphi_tot - mS * G * h_arm
        if denom > 100:
            roll_rad = (mS * ay * h_arm) / denom
        else:
            # 第 9 讲失稳判据：kphi_tot ≤ mS·g·h_arm（P-Δ 几何负刚度）→ 侧倾发散。
            # 防线保留（讲义钦定），但 F-16：显式上报，不再静默置零。 — lecture-9
            # instability criterion: kphi_tot ≤ mS·g·h_arm (P-Δ geometric negative
            # stiffness) -> roll divergence. The defense line stays (mandated by the
            # lecture) but F-16: report explicitly instead of silently zeroing.
            _roll_unstable = True
            roll_rad = 0.0
        dt_f = roll_rad * half_f    # 侧倾角×半轮距 = 轮行程差 mm — roll angle × half track = wheel-travel difference (mm)
        dt_r = roll_rad * half_r
        zrc_f = _interp_fb(swf["travel"], swf["rc_h"], dt_f, zrc_f0) / 1000.0
        zrc_r = _interp_fb(swr["travel"], swr["rc_h"], dt_r, zrc_r0) / 1000.0
        kwf = (_kw_at(vehicle.front, dt_f, kw_f0) + _kw_at(vehicle.front, -dt_f, kw_f0)) / 2.0
        kwr = (_kw_at(vehicle.rear, dt_r, kw_r0) + _kw_at(vehicle.rear, -dt_r, kw_r0)) / 2.0
        kphi_f = 0.5 * kwf * tf * tf + kphi_arb_f
        kphi_r = 0.5 * kwr * tr * tr + kphi_arb_r
        kphi_tot = kphi_f + kphi_r

    hra = zrc_f + (a / L) * (zrc_r - zrc_f)
    h_arm = max(0.05, hs - hra)
    denom = kphi_tot - mS * G * h_arm
    if denom > 100:
        roll_rad = (mS * ay * h_arm) / denom
        roll_grad = ((mS * G * h_arm) / denom) * R2D
    else:
        _roll_unstable = True
        roll_rad = 0.0
        roll_grad = 0.0
    if warnings is not None and _roll_unstable:
        warnings.append(
            "roll divergence: kphi_tot <= mS·g·h_arm (P-Δ geometric negative stiffness, "
            "lecture-9 criterion) — roll angle zeroed, load-transfer results for this "
            "case are not reliable")
    roll_deg = roll_rad * R2D

    F_aero = q.aero_force_n
    Fzf0 = mS * G * (b / L) + mU_f * G + F_aero * q.aero_bias
    Fzr0 = mS * G * (a / L) + mU_r * G + F_aero * (1 - q.aero_bias)

    dFz_u_f = mU_f * ay * (huf / tf)
    dFz_u_r = mU_r * ay * (hur / tr)
    dFz_geo_f = (mS * ay * (b / L)) * (zrc_f / tf)
    dFz_geo_r = (mS * ay * (a / L)) * (zrc_r / tr)
    dFz_elas_f = (kphi_f * roll_rad) / tf
    dFz_elas_r = (kphi_r * roll_rad) / tr
    dFz_f_tot = dFz_u_f + dFz_geo_f + dFz_elas_f
    dFz_r_tot = dFz_u_r + dFz_geo_r + dFz_elas_r

    sum_transfer = dFz_f_tot + dFz_r_tot
    # F-09（2026-08-30）：`sum_transfer > 1` 使左转（gy<0，sum<0）恒显示 50%。
    # 改 |sum|>1；50% 占位仅用于 ay≈0（讲义第九讲钦定约定，保留）。 — F-09 (2026-08-30):
    # `sum_transfer > 1` made left turns (gy<0, sum<0) always display 50%. Changed to
    # |sum|>1; the 50% placeholder remains only for ay≈0 (convention mandated by lecture
    # 9, kept).
    tlltd = (dFz_f_tot / sum_transfer) * 100 if abs(sum_transfer) > 1 else 50.0
    dFz_long = (mT * ax * (vehicle.hcg_mm / 1000.0)) / L

    sgn_y = 1.0 if q.gy >= 0 else -1.0
    fz = {
        "FL": max(0.0, (Fzf0 - dFz_long) / 2 - dFz_f_tot * sgn_y),
        "FR": max(0.0, (Fzf0 - dFz_long) / 2 + dFz_f_tot * sgn_y),
        "RL": max(0.0, (Fzr0 + dFz_long) / 2 - dFz_r_tot * sgn_y),
        "RR": max(0.0, (Fzr0 + dFz_long) / 2 + dFz_r_tot * sgn_y),
    }

    # ── 稳态不足转向梯度（线性区，镜像前端 solveQuasiStatic）──
    # α轴 = Fy轴 / ΣCα(各轮真实载荷)；Cα = B·C·D(Fz) 含 LS 载荷敏感性
    # （EP08/EP10：载荷转移使轴总刚度次线性增长 → 梯度含非线性饱和种子）。
    # 只含轮胎分量；roll steer / 柔度项在 sweep_roll / compliance 通道另查。
    # M5（2026-09-20）：Cα 再乘**按胎宽配平的轴系数** k = W_axle / mean(W_f, W_r)
    # （AxleSpec.tire_width_scale）——补上"后胎更宽 ⇒ 后轴更抓 ⇒ 不足转向"这条
    # 真车主平衡机制（此前 tire.W 从不进入刚度，模型对胎宽盲，导致只能靠
    # 前重配重才能凑出不足转向）。等宽或缺字段时 k≡1，旧行为逐位不变。
    # English: steady-state understeer gradient (linear region, mirrors the frontend
    # solveQuasiStatic): axle alpha = Fy_axle / ΣCα (per wheel's real load); Cα = B·C·D(Fz)
    # with LS load sensitivity (EP08/EP10: load transfer makes axle total stiffness grow
    # sub-linearly -> the gradient carries the seed of nonlinear saturation). Tire term
    # only; roll-steer / compliance terms are looked up in the sweep_roll / compliance
    # channels. M5 (2026-09-20): Cα additionally multiplied by the width-balanced axle
    # factor k = W_axle / mean(W_f, W_r) (AxleSpec.tire_width_scale) — capturing the
    # real-car balancing mechanism "wider rear tires => rear axle grips more =>
    # understeer" (previously tire.W never entered stiffness, the model was blind to tire
    # width, and only heavy front ballast could fake understeer). Equal widths or missing
    # fields => k≡1, behavior bit-for-bit unchanged.
    us_grad = alpha_f = alpha_r = 0.0
    if tire is not None and abs(q.gy) >= 0.02:
        k_f = vehicle.front.tire_width_scale(vehicle.rear)
        k_r = vehicle.rear.tire_width_scale(vehicle.front)
        # M8（2026-09-20）：把**整车静轮载均值 mT·g/4**作为载荷敏感性参考（单胎模型
        # 只有一个工作点口径）。此前 LS 一律相对 FzNom=3500 计算，而 FSAE 轮载仅约
        # 580 N ⇒ dμ/dFz 被稀释约 6 倍：实测 ARB 直径 0→30mm 时 kphi_f 470→3351
        # N·m/°、TLLTD 45.5%→74.9%、侧倾梯度 0.456→0.119 °/g，US 梯度却只从
        # 0.2104 动到 0.2262（+0.016 °/g）——载荷转移类杠杆形同虚设。改后同一处
        # 杠杆约 +0.083 °/g。取均值（而非按轴）是为了与前端 03-mechanism.js 及
        # 瞬态层 11-stages/transient 用**同一个数**，避免稳态/瞬态静默分叉。
        # 注意 FzNom 仍用于 D 的量级（μ_nom=Fy0/FzNom 标定不变）。 — M8 (2026-09-20): use
        # the vehicle mean static wheel load mT·g/4 as the load-sensitivity reference (the
        # single-tire model has only one operating point). Previously LS was always
        # computed relative to FzNom=3500 while a real FSAE wheel carries ~580 N =>
        # dμ/dFz diluted ~6x: measured ARB 0→30mm moved kphi_f 470→3351 N·m/°, TLLTD
        # 45.5%→74.9%, roll gradient 0.456→0.119 °/g, yet the US gradient only 0.2104→
        # 0.2262 (+0.016 °/g) — load-transfer levers were nearly useless. After the change
        # the same lever moves ~+0.083 °/g. Taking the mean (not per-axle) keeps the same
        # number as the frontend 03-mechanism.js and the transient layer (11-stages /
        # transient), avoiding a silent steady/transient divergence. Note FzNom is still
        # used for D's magnitude (μ_nom=Fy0/FzNom calibration unchanged).
        fz_ref_v = max(1.0, mT * G / 4.0)

        def _ca_mf(fz_w: float, k: float = 1.0) -> float:
            """侧偏刚度 Cα（N/rad）——委托 tire_mf 的唯一实现（M5 单源化）。

            此前这里是该公式的第三份拷贝（另两份在 tire_mf 的
            evaluate_magic_formula_fy 与 MagicFormulaSub._d）。
            M8：参考载荷取 `tire.FzRef`（请求显式给出时）否则整车静轮载均值。

            English: cornering stiffness Cα (N/rad) — delegated to the single
            implementation in tire_mf (M5 single-sourcing). Previously this was the third
            copy of the formula (the other two in tire_mf's evaluate_magic_formula_fy and
            MagicFormulaSub._d). M8: the reference load is `tire.FzRef` (when the request
            gives it explicitly) else the vehicle mean static wheel load.
            """
            return cornering_stiffness(fz_w, Fy0=tire.Fy0, By=tire.By,
                                       Cy=tire.Cy, FzNom=tire.FzNom,
                                       LS=tire.LS, axle_scale=k,
                                       FzRef=tire.FzRef if tire.FzRef else fz_ref_v)
        fy_f = mT * ay * (b / L)
        fy_r = mT * ay * (a / L)
        ca_f = _ca_mf(fz["FL"], k_f) + _ca_mf(fz["FR"], k_f)
        ca_r = _ca_mf(fz["RL"], k_r) + _ca_mf(fz["RR"], k_r)
        # ── M11（2026-09-20）· P12 外倾→抓地耦合 ────────────────────────────
        # 真车上 ARB 的另一条（常是主导的）平衡杠杆是"侧倾→外倾→抓地"，而旧模型
        # 的 US 梯度只含 Cα(Fz)：ARB 只能靠载荷敏感性间接影响平衡。实测前 ARB
        # 0→30mm 时 US 仅 +0.083 °/g（M8 修完参考载荷后），与真车"ARB 一动平衡
        # 就变"的直觉仍不符。
        #
        # 建模（线性区标准处理）：侧倾使轮对车身产生行程差 dt = roll_rad·半轮距
        # （上面已算出 dt_f/dt_r，原用于弹性载荷转移），经悬架外倾增益产生外倾变化，
        # 而外倾在零侧偏角下就有侧向力（外倾推力）：
        #     γ_w = camber_deg + (camber_gain°/25mm)/25 · dz_w
        #     Fy_cam,w = Cg · (−radians(γ_w)) · Fz_w      （外倾推力，朝弯心为正）
        # 于是轮到轴一级：α_axle = [Fy_axle − ΣFy_cam] / ΣCα
        # 符号自检：外轮（受载侧）在压缩行程、增益为负 ⇒ γ 更负 ⇒ (−γ)>0 ⇒ 贡献正
        # 侧向力（帮忙）；内轮反向 ⇒ 帮忙更少甚至帮倒忙。两侧之差正是"侧倾改变平衡"。
        #
        # 开关：两轴 `camber_gain_deg_per_25mm` 缺省 0 ⇒ 本项恒 0 ⇒ **历史行为逐位不变**
        # （直接调 API 的调用方不受影响）；前端把自身扫掠得到的 cg 随 payload 下发。
        # 只加推力项、不动 Cα：外倾对 Cα 是二阶小量，把两项都塞进来会让"这个数是
        # 哪来的"变得不可审计。 English: M11 (2026-09-20) · P12 camber-grip coupling: on a
        # real car the other (often dominant) ARB balancing lever is "roll -> camber ->
        # grip", while the old US gradient contained only Cα(Fz): the ARB could only
        # influence balance indirectly via load sensitivity. Measured: front ARB 0→30mm
        # moved US only +0.083 °/g even after the M8 reference-load fix, still at odds with
        # the intuition that "touch the ARB and balance changes".
        # Modeling (standard linear-region treatment): roll gives the wheels a travel
        # difference dt = roll_rad·half-track relative to the body (dt_f/dt_r computed
        # above for elastic load transfer), which through the suspension camber gain
        # creates camber change; camber produces lateral force even at zero slip angle
        # (camber thrust): γ_w = camber_deg + (camber_gain°/25mm)/25 · dz_w;
        # Fy_cam,w = Cg · (−radians(γ_w)) · Fz_w (thrust positive toward the turn center).
        # Wheel to axle: α_axle = [Fy_axle − ΣFy_cam] / ΣCα. Sign self-check: the outer
        # (loaded) wheel is in compression and the gain is negative => γ more negative =>
        # (−γ)>0 => contributes positive lateral force (helps); the inner wheel the other
        # way => helps less or even hurts. The two-side difference is exactly "roll changes
        # the balance".
        # Switch: both axles' `camber_gain_deg_per_25mm` default 0 => the term is always 0
        # => **historical behavior bit-for-bit unchanged** (direct API callers unaffected);
        # the frontend sends its own swept cg with the payload. Thrust term only, Cα
        # untouched: camber is a second-order effect on Cα, and folding both in would make
        # "where this number comes from" unauditable.
        cam_gain_f = vehicle.front.camber_gain_deg_per_25mm
        cam_gain_r = vehicle.rear.camber_gain_deg_per_25mm
        cam_f = cam_r = 0.0
        if tire.Cg and (cam_gain_f or cam_gain_r):
            def _camber_thrust(axle_camber, gain_per_25mm, dz_outer, fz_out, fz_in):
                """外轮 dz=+dt、内轮 dz=−dt；返回该轴外倾推力之和（N，朝弯心为正）。

                English: outer wheel dz=+dt, inner wheel dz=−dt; returns the axle's
                summed camber thrust (N, positive toward the turn center).
                """
                g_out = axle_camber + (gain_per_25mm / 25.0) * dz_outer
                g_in = axle_camber - (gain_per_25mm / 25.0) * dz_outer
                return (tire.Cg * (-math.radians(g_out)) * fz_out
                        + tire.Cg * (-math.radians(g_in)) * fz_in)
            # 受载侧：Gy>0 → FR；Gy<0 → FL。dt_f 的符号随 roll_rad（∝ay）自动跟随，
            # 故只需按"外=+dt / 内=−dt"配对，无需再乘 sgnY（乘了会双重取反）。 — loaded
            # side: gy>0 -> FR; gy<0 -> FL. dt_f's sign follows roll_rad (∝ay) automatically,
            # so just pair outer=+dt / inner=−dt; no extra sgnY multiply (it would
            # double-negate).
            if (q.gy or 0.0) >= 0:
                cam_f = _camber_thrust(vehicle.front.camber_deg, cam_gain_f, dt_f, fz["FR"], fz["FL"])
                cam_r = _camber_thrust(vehicle.rear.camber_deg, cam_gain_r, dt_r, fz["RR"], fz["RL"])
            else:
                cam_f = _camber_thrust(vehicle.front.camber_deg, cam_gain_f, dt_f, fz["FL"], fz["FR"])
                cam_r = _camber_thrust(vehicle.rear.camber_deg, cam_gain_r, dt_r, fz["RL"], fz["RR"])
        if ca_f > 1e-6 and ca_r > 1e-6:
            alpha_f = ((fy_f - cam_f) / ca_f) * R2D
            alpha_r = ((fy_r - cam_r) / ca_r) * R2D
            us_grad = (alpha_f - alpha_r) / q.gy   # deg/g（gy 以 g 为单位） — deg/g (gy is in g)
    elif tire is not None and warnings is not None and abs(q.gy) < 0.02:
        warnings.append("|gy|<0.02g: understeer gradient and front/rear slip angles not computed (divide-by-zero protection in the linear region)")

    # ── Jacking 效应（kinematics.jacking 同源，讲义 EP10）── ( jacking effect — same origin
    # as kinematics.jacking, lecture EP10 )
    f_jack_f = mS * ay * (b / L) * (2.0 * zrc_f / tf)
    f_jack_r = mS * ay * (a / L) * (2.0 * zrc_r / tr)
    f_jack = f_jack_f + f_jack_r
    jack_heave_mm = f_jack / ((kw_f0 + kw_r0) / 1000.0)   # 簧载 heave（mm） — sprung heave (mm)

    return ChassisLoads(
        roll_deg=roll_deg, roll_grad_deg_per_g=roll_grad,
        kphi_f=kphi_f * D2R, kphi_r=kphi_r * D2R, kphi_tot=kphi_tot * D2R,
        arb_share_f=(kphi_arb_f / kphi_f) * 100 if kphi_f > 0 else 0,
        arb_share_r=(kphi_arb_r / kphi_r) * 100 if kphi_r > 0 else 0,
        dFz_u_f=dFz_u_f, dFz_geo_f=dFz_geo_f, dFz_elas_f=dFz_elas_f, dFz_f_tot=dFz_f_tot,
        dFz_u_r=dFz_u_r, dFz_geo_r=dFz_geo_r, dFz_elas_r=dFz_elas_r, dFz_r_tot=dFz_r_tot,
        dFz_long=dFz_long, tlltd_pct=tlltd, fz={k: round(float(v), 2) for k, v in fz.items()},
        us_grad_deg_per_g=round(us_grad, 4),
        alpha_f_deg=round(alpha_f, 4), alpha_r_deg=round(alpha_r, 4),
        jacking_f_n=round(f_jack, 1), jacking_heave_mm=round(jack_heave_mm, 4),
    )


def _roll_steer_rate(vehicle: VehicleSpec) -> dict[str, float]:
    """侧倾转向率 @0（°toe/mm，外轮，±10mm 轮跳中央差分）。

    完整版不足转向特性的 roll steer 分量基底（讲义 EP02/EP13）：
    侧倾时轮对车身的附加转向角 δ_rs(roll) ≈ rate · roll_rad · half。

    English: roll-steer rate at 0 (°toe/mm, outer wheel, ±10mm central difference). The
    base of the roll-steer component of the full understeer characteristic (lectures
    EP02/EP13): under roll, the wheel's added steering angle relative to the body
    δ_rs(roll) ≈ rate · roll_rad · half.
    """
    corners = make_corners(vehicle)
    out: dict[str, float] = {}
    for ax_name, ax_key in (("front", "FR"), ("rear", "RR")):
        ax = vehicle.front if ax_name == "front" else vehicle.rear
        c = corners[ax_key]
        m_p, _, _ = corner_pose(ax, c["points"], 10.0, 0.0, c["left"])
        m_m, _, _ = corner_pose(ax, c["points"], -10.0, 0.0, c["left"])
        out[ax_name] = (m_p["toe"] - m_m["toe"]) / 20.0   # °/mm
    return out


def _us_curve(vehicle: VehicleSpec, tire, mr: dict[str, float],
              rcH: dict[str, float], rc_sw: dict[str, dict],
              roll_steer: dict[str, float] | None = None,
              n: int = 21, gy_max: float = 2.0) -> dict[str, list]:
    """不足转向特性 δ-ay 扫掠（底盘开发汇报核心图，讲义 EP11/EP12）。

    沿 gy 0→gy_max 均匀扫描准静态工况。完整版：
        δ_us = (αf − αr)  [轮胎分量]
             + (δ_rs,f − δ_rs,r)  [侧倾转向分量，rate·roll_rad·half]
    compliance steer 项在 compliance 通道另查，不在此伪造。
    输出同时携带分量（us_tire_deg / us_roll_steer_deg）供前端双线叠画。

    English: understeer characteristic δ-ay sweep (the core chassis-development report
    chart; lectures EP11/EP12). Sweeps quasi-static cases uniformly along gy 0→gy_max.
    Full version: δ_us = (αf − αr) [tire component] + (δ_rs,f − δ_rs,r) [roll-steer
    component; rate·roll_rad·half]. The compliance-steer term is looked up in the
    compliance channel, not faked here. The output carries the components
    (us_tire_deg / us_roll_steer_deg) so the frontend can overplot two lines.
    """
    from src.api.v3models import QuasiInputs
    half_f = abs(vehicle.front.points["WC"][0])    # mm
    half_r = abs(vehicle.rear.points["WC"][0])
    gy_l, af_l, ar_l, tire_l, rs_l, us_l, roll_l = [], [], [], [], [], [], []
    for i in range(n):
        g = gy_max * i / (n - 1)
        q = QuasiInputs(gy=g, gx=0.0, aero_force_n=0.0, aero_bias=0.5)
        ld = quasi_loads(vehicle, q, mr, rcH, rc_sw, warnings=None, tire=tire)
        roll_rad = math.radians(ld.roll_deg)
        rs = 0.0
        if roll_steer:
            rs_f = roll_steer.get("front", 0.0) * roll_rad * half_f
            rs_r = roll_steer.get("rear", 0.0) * roll_rad * half_r
            rs = rs_f - rs_r
        tire_us = round(ld.alpha_f_deg - ld.alpha_r_deg, 4) if g >= 0.02 else 0.0
        gy_l.append(round(g, 4))
        af_l.append(ld.alpha_f_deg)
        ar_l.append(ld.alpha_r_deg)
        tire_l.append(tire_us)
        rs_l.append(round(rs, 4))
        us_l.append(round(tire_us + rs, 4))
        roll_l.append(ld.roll_deg)
    return {"gy": gy_l, "alpha_f_deg": af_l, "alpha_r_deg": ar_l,
            "us_tire_deg": tire_l, "us_roll_steer_deg": rs_l,
            "us_angle_deg": us_l, "roll_deg": roll_l}


def _resolve_mr_fb(ax: AxleSpec, points: dict, is_front: bool,
                   warnings: list[str]) -> tuple[float, bool]:
    """MR 三路优先级（F-19/F-67）：显式参数 > mr_at_zero 数值推导 > 分轴常量。

    返回 (mr, 是否回退常量)。回退常量分轴（前 0.75 / 后 0.78——旧实现恒
    0.75 使后轴也吃到前轴常量），且异常原因随 warning 上报（不再静默吞）。

    M5（2026-09-20）**MR 口径分歧显式化**：前端恒定下发自己的扫掠 MR
    （web/js/06-ui-panels.js `motion_ratio: SIM.mrRefF||0.75`，其值是前端真实
    几何扫掠得到的 mr@0，不是占位常量）⇒ 显式分支永远命中，引擎的
    `mr_at_zero` 推导在**生产链路上从未被执行**，两口径的分叉（实测前
    0.47 vs 0.75、后 0.17 vs 0.78，OpenItem-B 摇臂双根分支）因此永久静默。
    本函数现在在命中显式值时**仍然算一遍推导并比对**，相对差 >15% 即上报
    warning——数值行为不变（显式值仍是权威），只是把分叉暴露在响应的
    warnings 里（前端会显示在状态行）。

    English: MR three-way priority (F-19/F-67): explicit parameter > mr_at_zero numerical
    derivation > per-axle constants. Returns (mr, fell back to constant). The fallback
    constants are per-axle (front 0.75 / rear 0.78 — the old implementation used 0.75 for
    both, so the rear axle also got the front constant), and the exception reason is
    reported via warning (no longer silently swallowed).
    M5 (2026-09-20) **MR accounting discrepancy made explicit**: the frontend always
    sends its own sweep MR (web/js/06-ui-panels.js `motion_ratio: SIM.mrRefF||0.75`, the
    mr@0 from the frontend's real geometry sweep, not a placeholder constant) => the
    explicit branch always hit, the engine's `mr_at_zero` derivation never ran on the
    production path, so the two-accounting divergence (measured front 0.47 vs 0.75, rear
    0.17 vs 0.78; OpenItem-B rocker dual-root branch) stayed silent forever. This function
    now computes the derivation even when the explicit value hits and compares it; a
    relative difference >15% raises a warning — numeric behavior unchanged (the explicit
    value remains authoritative); the divergence is only exposed in the response warnings
    (the frontend shows them in the status line).
    """
    if ax.motion_ratio is not None:
        mr = round(float(ax.motion_ratio), 4)
        try:
            derived = round(mr_at_zero(ax, points, False), 4)
        except Exception:  # noqa: BLE001
            derived = None   # 推导失败不影响显式值；兜底告警由无显式分支负责 — derivation failure does not affect the
            # explicit value; the fallback warning is owned by the no-explicit branch
        if derived:
            rel = abs(mr - derived) / max(mr, derived, 1e-9)
            if rel > 0.15:
                warnings.append(
                    f"MR discrepancy ({'front' if is_front else 'rear'} axle): frontend explicit {mr} "
                    f"vs engine-derived {derived} (relative diff {rel * 100:.0f}%) -> using the explicit value; "
                    f"root cause tracked as OpenItem-B (rocker dual-root branch selection)")
        return mr, False
    try:
        v = round(mr_at_zero(ax, points, False), 4)
        return v, False
    except Exception as exc:  # noqa: BLE001
        c = 0.75 if is_front else 0.78
        warnings.append(f"MR compute failed for {'front' if is_front else 'rear'}: "
                        f"{exc.__class__.__name__} {exc} -> falling back to constant {c}")
        return c, True


def solve_chassis(req: ChassisRequest) -> ChassisPoseResponse:
    """整车单点：四角运动学 + 姿态 + 准静态载荷。"""
    t0 = time.perf_counter()
    corners = make_corners(req.vehicle)
    tr = req.travel
    trav = {"FL": tr.fl, "FR": tr.fr, "RL": tr.rl, "RR": tr.rr}
    pose: dict[str, dict] = {}
    warnings: list[str] = []
    statuses: set[str] = set()
    rcH: dict[str, float] = {}
    # MR 链（P2，2026-08-22；F-67 提取模块级）：显式参数 > 引擎数值推导
    # mr_at_zero（rocker 真实三维轴 + strut_attach 对齐后恢复可信）> 前端
    # 同源常量回退（SIM.mrRefF||0.75 / mrRefR||0.78，仅解算异常时使用）。 — MR chain (P2,
    # 2026-08-22; extracted module-level by F-67): explicit parameter > engine numerical
    # derivation mr_at_zero (restored to trustworthy after the rocker real 3D axis +
    # strut_attach alignment) > frontend same-source constant fallback
    # (SIM.mrRefF||0.75 / mrRefR||0.78, used only when solving fails).
    mr_f, _fb_f = _resolve_mr_fb(req.vehicle.front, req.vehicle.front.points,
                                 is_front=True, warnings=warnings)
    mr_r, _fb_r = _resolve_mr_fb(req.vehicle.rear, req.vehicle.rear.points,
                                 is_front=False, warnings=warnings)
    mr = {"front": round(mr_f, 4), "rear": round(mr_r, 4)}

    for key in CORNER_KEYS:
        ax = req.vehicle.front if key in ("FL", "FR") else req.vehicle.rear
        c = corners[key]
        m, res, warn = corner_pose(ax, c["points"], trav[key],
                                   req.rack if key in ("FL", "FR") else 0.0, c["left"])
        statuses.add("VALID" if res < 0.02 else "SOLVER_FAILED")
        warnings.extend(warn)
        out = {k: (None if v is None else round(float(v), 6))
               for k, v in m.items() if k != "residual"}
        out["residual"] = round(res, 6)
        pose[key] = out
        if m["rc_h"] is not None:
            rcH[key] = float(m["rc_h"])

    rc_sw = {"front": axle_rc_sweep(req.vehicle.front),
             "rear": axle_rc_sweep(req.vehicle.rear)}
    # 守恒记账检查（2026-08-22）：hs 与 hcg 是独立输入，若
    # (mS·hs + mU·hu)/mT ≠ hcg，轴转移之和与整车力矩公式不严格闭合（第 9 讲发现）。 —
    # conservation bookkeeping check (2026-08-22): hs and hcg are independent inputs; if
    # (mS·hs + mU·hu)/mT ≠ hcg, the axle-transfer sum and the full-vehicle moment formula
    # do not close exactly (found during lecture-9 work).
    v_ = req.vehicle
    h_weighted = (v_.sprung_mass_kg * v_.hs_mm
                  + v_.front.unsprung_kg * 2 * v_.front.tire_radius
                  + v_.rear.unsprung_kg * 2 * v_.rear.tire_radius) / v_.mass_kg
    if abs(h_weighted - v_.hcg_mm) > 20.0:
        warnings.append(f"inconsistent hs/hcg accounting: weighted height {h_weighted:.0f}mm vs hcg "
                        f"{v_.hcg_mm:.0f}mm (load-transfer sums will deviate from the full-vehicle formula)")
    loads = quasi_loads(req.vehicle, req.quasi, mr, rcH, rc_sw, warnings=warnings,
                        tire=req.tire)
    us_curve = _us_curve(req.vehicle, req.tire, mr, rcH, rc_sw,
                         roll_steer=_roll_steer_rate(req.vehicle))
    attitude = _attitude(trav, req.vehicle)
    return ChassisPoseResponse(
        status="SOLVER_FAILED" if "SOLVER_FAILED" in statuses else "VALID",
        ms=(time.perf_counter() - t0) * 1000.0,
        pose=pose, attitude=attitude, loads=loads, mr=mr, us_curve=us_curve,
        warnings=list(dict.fromkeys(warnings))[:8],
    )


def _attitude(trav: dict[str, float], vehicle: VehicleSpec) -> dict[str, float]:
    """由四轮轮跳推导整车姿态（heave/roll/pitch，同 core/models derived_pose）。

    English: derive full-vehicle attitude from the four wheel travels (heave/roll/pitch,
    same as core/models derived_pose).
    """
    t = trav
    heave = (t["FL"] + t["FR"] + t["RL"] + t["RR"]) / 4.0
    track_avg = (abs(vehicle.front.points["WC"][0]) * 2
                 + abs(vehicle.rear.points["WC"][0]) * 2) / 2.0
    roll = math.degrees(math.atan2(((t["FL"] + t["RL"]) - (t["FR"] + t["RR"])) / 2.0,
                                   track_avg))
    pitch = math.degrees(math.atan2(((t["FL"] + t["FR"]) - (t["RL"] + t["RR"])) / 2.0,
                                    vehicle.wheelbase_mm))
    return {"heave_mm": round(heave, 4), "roll_deg": round(roll, 4),
            "pitch_deg": round(pitch, 4)}


def _merged_status(ss: set[str]) -> str:
    return "SOLVER_FAILED" if "SOLVER_FAILED" in ss else "VALID"


def sweep_bump(req: ChassisRequest) -> ChassisSweepResponse:
    """整车平行轮跳扫掠：四轮同向 ±travel。

    English: full-vehicle parallel bump sweep — all four wheels ±travel in phase.
    """
    from src.metrics.kandc import bump_steer_deg_per_25, camber_gain_deg_per_25

    t0 = time.perf_counter()
    corners = make_corners(req.vehicle)
    xs = np.linspace(req.sweep.min, req.sweep.max, req.sweep.n)
    curves: dict[str, list] = {"travel": [round(float(v), 4) for v in xs]}
    statuses: set[str] = set()
    warnings: list[str] = []
    for t in xs:
        for key in CORNER_KEYS:
            ax = req.vehicle.front if key in ("FL", "FR") else req.vehicle.rear
            c = corners[key]
            m, res, warn = corner_pose(ax, c["points"], float(t), 0.0, c["left"])
            statuses.add("VALID" if res < 0.02 else "SOLVER_FAILED")
            warnings.extend(warn)
            for kk in ("cam", "toe"):
                curves.setdefault(f"{kk}_{key}", []).append(round(float(m[kk]), 6))
    gains: dict[str, float] = {}
    for side, key in (("front", "FR"), ("rear", "RR")):
        gains[f"camber_gain_{side}"] = camber_gain_deg_per_25(
            curves[f"cam_{key}"], curves["travel"], at=0.0)
        gains[f"bump_steer_{side}"] = bump_steer_deg_per_25(
            curves[f"toe_{key}"], curves["travel"], at=0.0)
    return ChassisSweepResponse(
        case="bump", status=_merged_status(statuses),
        ms=(time.perf_counter() - t0) * 1000.0,
        curves=curves, gains=gains, warnings=list(dict.fromkeys(warnings))[:8],
    )


def sweep_roll(req: ChassisRequest) -> ChassisSweepResponse:
    """整车侧倾扫掠：左轮 −t / 右轮 +t（前后同幅）。

    English: full-vehicle roll sweep — left wheel −t / right wheel +t (same amplitude
    front and rear).
    """
    t0 = time.perf_counter()
    corners = make_corners(req.vehicle)
    xs = np.linspace(req.sweep.min, req.sweep.max, req.sweep.n)
    curves: dict[str, list] = {"travel": [], "roll_deg": []}
    statuses: set[str] = set()
    warnings: list[str] = []
    track_avg = (abs(req.vehicle.front.points["WC"][0]) * 2
                 + abs(req.vehicle.rear.points["WC"][0]) * 2) / 2.0
    for t in xs:
        roll_deg = math.degrees(math.atan2(2.0 * t, track_avg))
        curves["travel"].append(round(float(t), 4))
        curves["roll_deg"].append(round(float(roll_deg), 5))
        for key, tt in (("FL", -t), ("FR", t), ("RL", -t), ("RR", t)):
            ax = req.vehicle.front if key in ("FL", "FR") else req.vehicle.rear
            c = corners[key]
            m, res, warn = corner_pose(ax, c["points"], float(tt), 0.0, c["left"])
            statuses.add("VALID" if res < 0.02 else "SOLVER_FAILED")
            warnings.extend(warn)
            for kk in ("cam", "toe"):
                curves.setdefault(f"{kk}_{key}", []).append(round(float(m[kk]), 6))
    gains: dict[str, float] = {}
    rd = curves["roll_deg"]
    if rd and abs(rd[-1] - rd[0]) > 1e-9:
        for side, key in (("front", "FR"), ("rear", "RR")):
            gains[f"roll_camber_gain_{side}"] = round(float(
                _slope(curves[f"cam_{key}"], rd, 0.0)), 6)
            gains[f"roll_steer_{side}"] = round(float(
                _slope(curves[f"toe_{key}"], rd, 0.0)), 6)
    return ChassisSweepResponse(
        case="roll", status=_merged_status(statuses),
        ms=(time.perf_counter() - t0) * 1000.0,
        curves=curves, gains=gains, warnings=list(dict.fromkeys(warnings))[:8],
    )


def sweep_steer(req: ChassisRequest) -> ChassisSweepResponse:
    """整车转向扫掠：rack 位移扫描（前轴两轮；后轮 rack=0）。

    gains 额外输出转向外倾增益分解（讲义 EP04×EP06：Caster 转向时给外轮
    负 camber、KPI 反向夺走；@最大转向角处：合成 ≈ −caster·sin δ + kpi·(1−cos δ)
    + 残差（连杆几何/摆臂高阶效应）。合成值来自真实机构求解曲线。

    English: full-vehicle steering sweep — rack displacement scan (front-axle two wheels;
    rear rack=0). gains additionally output the steer-camber-gain decomposition (lectures
    EP04×EP06: caster steers the outer wheel negative camber while KPI takes some back;
    at maximum steering angle the total ≈ −caster·sin δ + kpi·(1−cos δ) + residual
    (linkage geometry / higher-order arm effects). The total comes from the real
    mechanism-solved curves.
    """
    t0 = time.perf_counter()
    corners = make_corners(req.vehicle)
    xs = np.linspace(req.sweep.min, req.sweep.max, req.sweep.n)
    curves: dict[str, list] = {"rack": [round(float(v), 4) for v in xs]}
    statuses: set[str] = set()
    warnings: list[str] = []
    for rk in xs:
        for key in ("FL", "FR"):
            ax = req.vehicle.front
            c = corners[key]
            m, res, warn = corner_pose(ax, c["points"], 0.0, float(rk), c["left"])
            statuses.add("VALID" if res < 0.02 else "SOLVER_FAILED")
            warnings.extend(warn)
            for kk in ("cam", "toe", "cast", "kpi"):
                curves.setdefault(f"{kk}_{key}", []).append(round(float(m[kk]), 6))
    gains: dict[str, float] = {}
    if len(xs) >= 3:
        gains["steer_toe_gain_deg_per_mm"] = round(
            float(np.interp(0.0 + 2.0, xs, curves["toe_FR"])
                  - np.interp(0.0 - 2.0, xs, curves["toe_FR"])) / 4.0, 6)
        # 转向外倾增益分解（δ 用内轮 toe 相对直行的变化近似转向角） — steer camber gain
        # decomposition (δ approximated by the inner-wheel toe change relative to straight)
        toe0 = float(np.interp(0.0, xs, curves["toe_FR"]))
        cam0 = float(np.interp(0.0, xs, curves["cam_FR"]))
        deltas = [v - toe0 for v in curves["toe_FR"]]
        imax = int(np.argmax(np.abs(deltas)))
        dmax = deltas[imax]
        cast0 = float(np.interp(0.0, xs, curves["cast_FR"]))
        kpi0 = float(np.interp(0.0, xs, curves["kpi_FR"]))
        total = curves["cam_FR"][imax] - cam0
        d_rad = math.radians(abs(dmax))
        caster_term = -cast0 * math.sin(d_rad)
        kpi_term = kpi0 * (1.0 - math.cos(d_rad))
        gains["steer_camber_gain_deg_per_deg"] = round(
            float(_slope(curves["cam_FR"], deltas, 0.0)), 6)
        gains["steer_camber_total_at_max_deg"] = round(total, 4)
        gains["steer_camber_caster_at_max_deg"] = round(caster_term, 4)
        gains["steer_camber_kpi_at_max_deg"] = round(kpi_term, 4)
        gains["steer_camber_residual_at_max_deg"] = round(
            total - caster_term - kpi_term, 4)
    return ChassisSweepResponse(
        case="steer", status=_merged_status(statuses),
        ms=(time.perf_counter() - t0) * 1000.0,
        curves=curves, gains=gains, warnings=list(dict.fromkeys(warnings))[:8],
    )
