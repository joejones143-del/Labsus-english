"""6DOF 橡胶衬套元件：逐自由度刚度/阻尼曲线 + 耦合项 + 预紧（S1 首批线性/查表/样条）。

使用状态（2026-08-22 登记）：S1 K&C 主路径仅使用平移刚度（kT）+ 预紧；
cT/cR 阻尼为存储占位（静态求解不使用）；coupled 6x6 耦合矩阵虽在
bush_force / bush_force_jac 中完整生效，但尚未有工况向求解器注入非零
耦合项（compliance.solve_compliance 的 S1 外载仅平移 3 分量、旋转 DOF 无
外部力矩 —— 见 M5 注释）。耦合/阻尼/旋转力矩属于 S3+ 载荷阶段能力。

6DOF rubber bushing element: per-DOF stiffness/damping curves + coupling
terms + preload (S1 first batch: linear / table / spline).

Usage status (registered 2026-08-22): the S1 K&C main path only uses the
translational stiffness (kT) + preload; cT/cR damping is stored as a
placeholder (not used by the static solve); although the coupled 6x6
coupling matrix is fully active in bush_force / bush_force_jac, no case
yet injects non-zero coupling terms into the solver (the S1 external
load of compliance.solve_compliance is only a 3-component translation;
the rotational DOFs have no external torque — see the M5 note).
Coupling / damping / rotational torques belong to S3+ load-stage
capability.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.interpolate import PchipInterpolator


@dataclass
class Curve:
    kind: str                       # "linear" | "table"（力-位移表）| "spline" — "linear" | "table" (force-displacement table) | "spline"
    k: float = 0.0                  # linear 刚度（N/mm 或 N·mm/rad） — linear stiffness (N/mm or N·mm/rad)
    xs: list[float] = field(default_factory=list)
    ys: list[float] = field(default_factory=list)

    def build(self):
        if self.kind == "linear":
            return lambda x: self.k * x, lambda x: np.full_like(np.asarray(x, float), self.k)
        if self.kind in ("table", "spline"):
            x = np.asarray(self.xs, float)
            y = np.asarray(self.ys, float)
            if len(x) < 2:
                raise ValueError(f"table/spline curve requires >=2 points, got {len(x)}")
            # F-66（2026-08-30）：x 非严格递增 → PchipInterpolator 静默抛错（仅 422
            # 兜底）。改为显式校验并给出明确错误信息。
            # F-66 (2026-08-30): non-strictly-increasing x → PchipInterpolator
            # raises an opaque error (caught only as a generic 422). Replace
            # with an explicit validation and a clear error message.
            if np.any(np.diff(x) <= 0):
                raise ValueError(
                    f"curve xs must be strictly increasing, got non-monotonic: {x.tolist()}")
            lo, hi = float(x[0]), float(x[-1])
            p = PchipInterpolator(x, y)
            # F-66（2026-08-30）：越界不再 np.clip —— 旧实现 clip 后表外刚度按端点
            # 值恒等（导数≈0），衬套表现为"屈服变软"（物理相反，且无警告）。
            # 改为端点斜率线性外推：表外仍保持端点刚度 k0/k1（硬化持续），
            # 边界行为物理连续（f(lo)=y0、f'(lo)=k0）。
            # F-66 (2026-08-30): out-of-range inputs are no longer np.clip'd —
            # the old implementation clamped, making the out-of-table stiffness
            # constant at the endpoint value (derivative ≈ 0), so the bushing
            # behaved as "yielding and softening" (physically backwards, and
            # with no warning). Instead, extrapolate linearly with the
            # endpoint slope: beyond the table the endpoint stiffness k0/k1
            # persists (hardening continues), and the boundary behavior is
            # physically continuous (f(lo)=y0, f'(lo)=k0).
            k0 = float(p.derivative()(lo))
            k1 = float(p.derivative()(hi))
            y0, y1 = float(y[0]), float(y[-1])
            def _f(v):
                v = np.asarray(v, float)
                return np.where(v < lo, y0 + k0 * (v - lo),
                                np.where(v > hi, y1 + k1 * (v - hi),
                                         p(np.clip(v, lo, hi))))
            def _d(v):
                v = np.asarray(v, float)
                return np.where(v < lo, k0,
                                np.where(v > hi, k1,
                                         p.derivative()(np.clip(v, lo, hi))))
            return _f, _d
        raise ValueError(f"unknown curve kind {self.kind}")


@dataclass
class Bushing6DOF:
    name: str
    anchor: np.ndarray              # 车身侧锚点（3,）——部件侧 = anchor + 位移分量 + 小角旋转 — chassis-side anchor (3,) — component side = anchor + displacement + small-angle rotation
    kT: list[Curve]                 # 3 个平移刚度 — 3 translational stiffnesses
    kR: list[Curve]                 # 3 个旋转刚度 — 3 rotational stiffnesses
    cT: list[float]                 # 3 个平移阻尼（速度相关项，S1 存储占位） — 3 translational dampings (velocity-dependent; stored placeholder in S1)
    cR: list[float]
    preload: np.ndarray             # 6 预紧力/矩 — 6 preload forces/torques
    coupled: np.ndarray | None = None  # 6x6 耦合刚度（可选） — 6x6 coupled stiffness (optional)
    member_nodes: list[str] = field(default_factory=list)            # 装配期：{衬套名: 节点名} 由求解器填入 — at assembly: {bushing name: node name}, filled in by the solver
    member_p0: dict[str, np.ndarray] = field(default_factory=dict)   # {节点名: 设计位置} — {node name: design position}

    def __post_init__(self):
        if len(self.kT) != 3:
            raise ValueError(f"kT must have length 3, got {len(self.kT)}")
        if len(self.kR) != 3:
            raise ValueError(f"kR must have length 3, got {len(self.kR)}")
        preload = np.asarray(self.preload, float)
        if preload.shape != (6,):
            raise ValueError(f"preload must have shape (6,), got {preload.shape}")
        if self.coupled is not None:
            coupled = np.asarray(self.coupled, float)
            if coupled.shape != (6, 6):
                raise ValueError(f"coupled must have shape (6,6), got {coupled.shape}")
        self._fT = [c.build() for c in self.kT]
        self._fR = [c.build() for c in self.kR]


def bush_force(b: Bushing6DOF, delta: np.ndarray) -> np.ndarray:
    """delta = [dx,dy,dz, rx,ry,rz]（小角），返回 6 抗力（力 N / 力矩 N·mm）。

    delta = [dx,dy,dz, rx,ry,rz] (small angles); returns the 6 reaction
    components (forces N / torques N·mm).
    """
    d = np.asarray(delta, float)
    f = np.zeros(6)
    for i in range(3):
        f[i] = b._fT[i][0](d[i])
    for i in range(3):
        f[3 + i] = b._fR[i][0](d[3 + i])
    if b.coupled is not None:
        f = f + np.asarray(b.coupled, float) @ d
    return f + np.asarray(b.preload, float)


def bush_force_jac(b: Bushing6DOF, delta: np.ndarray) -> np.ndarray:
    """解析（含样条导数）：∂f/∂δ（6x6）。

    Analytic (including spline derivatives): ∂f/∂δ (6x6).
    """
    d = np.asarray(delta, float)
    J = np.zeros((6, 6))
    for i in range(3):
        J[i, i] = b._fT[i][1](d[i])
    for i in range(3):
        J[3 + i, 3 + i] = b._fR[i][1](d[3 + i])
    if b.coupled is not None:
        J = J + np.asarray(b.coupled, float)
    return J
