"""差速器模型（M16 / G46，2026-09-20）

背景
────
此前驱动扭矩分配是**逐轮独立**算的：`fx_w = min(0.75·μFz_w, f_avail·share)·0.5`
—— 左右永远 50/50，且每轮各自可用满自身附着。这既不是开放式、也不是锁死式，
而是一个"能无限转移扭矩却仍保持 50/50"的数学便利物。后果：

* **开放式差速器的核心失效模式没建模**：弱侧轮（弯内、抬轮、湿滑）打滑时整轴驱动力
  被它拖死（`F_axle ≤ 2·min(cap_L, cap_R)`）。这是 FSAE 出弯最要紧的一条。
* **差速器对弯中平衡的作用完全缺失**：锁死/限滑把更多扭矩给受载侧外轮 ⇒ 产生指向弯外
  的横摆力矩 ⇒ 偏转向过度；开放式反过来。这恰是"弯中平衡的另一半"。
  （本项目已有：前/后驱动比例、轮胎载荷敏感性、外倾耦合；差速器是缺的那块。）

模型（轴级，力单位 N，避免扭矩/半径来回换算出错）
────────────────────────────────────────────────
单轮纵向附着上限 `cap_w`（N）。差速器用"**锁紧力差上限**"刻画能转移多少：

    kind=open   → ΔF_max = 0                      （左右必须等扭矩）
    kind=locked → ΔF_max = ∞                      （两轮同速，可达各自附着上限）
    kind=lsd    → ΔF_max = preload/r + bias·F_in  （锁紧扭矩 T_lock = preload + bias·T_in）

于是整轴可传递力与左右分配为闭式解：

    lo, hi  = min(cap), max(cap)
    F_total = min(F_avail, lo + min(hi, lo + ΔF_max))
    F_hi    = min(hi, (F_total + ΔF_max)/2)      ,   F_hi ≥ F_total/2
    F_lo    = F_total − F_hi

自检：open（ΔF=0）⇒ F_total=2·lo、F_hi=lo（严格 50/50，弱侧决定整轴）✓
      locked（ΔF=∞）⇒ F_total=lo+hi、F_hi=hi（按附着分配，两轮各达上限）✓
      对称工况 cap_l=cap_r ⇒ 三种差速器给出**完全相同**的结果（各 50%）✓ 无伪差异

分工说明
────────
横摆力矩**不单独算**：分配决定左右纵向力 `fx`，而 `derivs` 里 `Mz += lx·fy − ly·fx`
已自动把它计入 —— 少一处口径就少一处漂移。

Differential model (M16 / G46, 2026-09-20)

Background
──────────
Drive torque used to be allocated **per wheel independently**:
`fx_w = min(0.75·μFz_w, f_avail·share)·0.5`
— left/right were always 50/50, and each wheel could use its full own
grip. That is neither open nor locked, but a mathematical convenience
that "can transfer unlimited torque while still keeping 50/50".
Consequences:

* **The open differential's core failure mode was not modeled**: when
  the weak-side wheel (inner in the corner, lifted, wet) spins, the
  whole axle's drive force is dragged down by it
  (`F_axle ≤ 2·min(cap_L, cap_R)`). This is the most critical FSAE
  corner-exit issue.
* **The differential's effect on mid-corner balance was entirely
  missing**: locked/LSD gives more torque to the loaded outer wheel ⇒ a
  yaw moment pointing out of the corner ⇒ an oversteering tendency;
  open is the opposite. This is exactly "the other half of mid-corner
  balance". (This project already has: front/rear drive split, tire
  load sensitivity, camber coupling; the differential was the missing
  piece.)

Model (axle level, forces in N to avoid torque/radius conversion errors)
────────────────────────────────────────────────
Single-wheel longitudinal grip cap `cap_w` (N). The differential
characterizes how much can be transferred via a "**maximum locked force
difference**":

    kind=open   → ΔF_max = 0                      (left/right torques must be equal)
    kind=locked → ΔF_max = ∞                      (both wheels at the same speed; each can reach its grip cap)
    kind=lsd    → ΔF_max = preload/r + bias·F_in  (locking torque T_lock = preload + bias·T_in)

The axle's transmittable force and the left/right split then have a
closed-form solution:

    lo, hi  = min(cap), max(cap)
    F_total = min(F_avail, lo + min(hi, lo + ΔF_max))
    F_hi    = min(hi, (F_total + ΔF_max)/2)      ,   F_hi ≥ F_total/2
    F_lo    = F_total − F_hi

Self-check: open (ΔF=0) ⇒ F_total=2·lo, F_hi=lo (strict 50/50; the weak
side limits the axle) ✓
      locked (ΔF=∞) ⇒ F_total=lo+hi, F_hi=hi (split by grip; both wheels
      reach their caps) ✓
      symmetric case cap_l=cap_r ⇒ all three differentials give
      **exactly identical** results (50% each) ✓ no spurious differences

Division of labor
────────────────
The yaw moment is **not computed separately**: the split determines the
left/right longitudinal forces `fx`, and `derivs`' `Mz += lx·fy − ly·fx`
already accounts for it — one fewer convention means one fewer place to
drift.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

KIND_OPEN = "open"
KIND_LSD = "lsd"
KIND_LOCKED = "locked"
_KINDS = (KIND_OPEN, KIND_LSD, KIND_LOCKED)


@dataclass(frozen=True)
class Differential:
    """差速器规格。力单位为 N、扭矩 N·m、半径 mm（与项目其余部分一致）。

    Differential specification. Forces in N, torques in N·m, radii in mm
    (consistent with the rest of the project).
    """

    kind: str = KIND_LOCKED
    preload_nm: float = 0.0        # 预载扭矩（LSD）：静态就有、与输入扭矩无关的锁紧 — preload torque (LSD): locking present statically, independent of the input torque
    bias_ratio: float = 0.0        # 扭矩偏置系数：T_lock = preload + bias_ratio·T_in — torque bias ratio: T_lock = preload + bias_ratio·T_in
    radius_mm: float = 325.0       # 该轴轮胎半径（扭矩↔力 折算） — this axle's tire radius (torque↔force conversion)

    def __post_init__(self) -> None:
        if self.kind not in _KINDS:
            raise ValueError(f"differential: invalid kind {self.kind} (expected one of {_KINDS})")
        if self.preload_nm < 0.0:
            raise ValueError(f"preload must not be negative ({self.preload_nm})")
        if self.bias_ratio < 0.0:
            raise ValueError(f"bias_ratio must not be negative ({self.bias_ratio})")
        if self.radius_mm <= 0.0:
            raise ValueError(f"tire radius must be positive ({self.radius_mm})")

    @property
    def radius_m(self) -> float:
        return self.radius_mm / 1000.0

    def lock_force_n(self, f_in_n: float) -> float:
        """允许的**左右纵向力差**上限（N）。

        Upper bound of the **allowed left/right longitudinal force
        difference** (N).
        """
        if self.kind == KIND_OPEN:
            return 0.0
        if self.kind == KIND_LOCKED:
            return math.inf
        return self.preload_nm / self.radius_m + self.bias_ratio * max(0.0, f_in_n)

    def axle_capacity_n(self, cap_l: float, cap_r: float, f_in_n: float) -> float:
        """该轴在这种差速器下能传出的最大纵向力（N）。

        Maximum longitudinal force this axle can transmit with this kind
        of differential (N).
        """
        lo, hi = min(cap_l, cap_r), max(cap_l, cap_r)
        return lo + min(hi, lo + self.lock_force_n(f_in_n))


def axle_static_capacity_split(cap_l: float, cap_r: float, f_avail: float,
                               diff: Differential) -> tuple[float, float]:
    """**准静态能力评估工具**（不是运行时分配）：力单位，问"这根轴最多能把多少驱动力放到地上"。

    用途：方案对比 / 报告 / 灵敏度分析里回答"开放式受弱侧拖累到什么程度"。
    ⚠ **运行时分配的真源是 `distribute_axle`**（扭矩单位，逐字移植自前端
    `web/js/16-powertrain.js::distributeAxle`）——两者形式不同、不要混用：
    本函数是**几何/附着上限**的静态估算，`distribute_axle` 是**扭矩转移**的运行时模型。

    **Quasi-static capability assessment tool** (not the runtime
    allocation): force units; asks "at most how much drive force can
    this axle put on the ground".

    Purpose: answering "how badly does the open differential get dragged
    down by the weak side" in design comparison / reports / sensitivity
    analysis.
    ⚠ **The runtime allocation source of truth is `distribute_axle`**
    (torque units, ported verbatim from the frontend
    `web/js/16-powertrain.js::distributeAxle`) — the two have different
    forms and must not be mixed: this function is a static estimate of
    the **geometric/grip caps**, while `distribute_axle` is the runtime
    **torque-transfer** model.
    """
    cap_l = max(0.0, float(cap_l))
    cap_r = max(0.0, float(cap_r))
    f_avail = max(0.0, float(f_avail))
    lo, hi = min(cap_l, cap_r), max(cap_l, cap_r)
    d_lock = diff.lock_force_n(f_avail)
    total = min(f_avail, lo + min(hi, lo + d_lock))
    # 分配 = 在「50/50（完全不能转移）」与「按附着比例（完全锁定）」之间按**可实现的力差**插值：
    #     Δ_ideal = total·(hi−lo)/(hi+lo)   ← 完全锁定时两侧应有的力差（正比于附着）
    #     Δ       = min(Δ_ideal, Δ_lock)    ← 差速器实际能实现多少
    #     f_hi = total/2 + Δ/2 ,  f_lo = total/2 − Δ/2
    # ⚠ 初版写的是"锁死时直接 f_hi = hi"，在小可用力下会把**全部**力给一侧（f_lo 归零）
    #   —— 对称工况断言当场抓出。插值式同时满足两端：
    #   对称工况 hi=lo ⇒ Δ_ideal=0 ⇒ 严格 50/50（三种差速器完全一致，无伪差异）；
    #   满载且锁死 total=lo+hi ⇒ Δ=hi−lo ⇒ f_hi=hi、f_lo=lo（各达自身附着上限）。
    # Allocation = interpolation between "50/50 (no transfer possible)" and
    # "by-grip proportions (fully locked)" based on the **achievable force
    # difference**:
    #     Δ_ideal = total·(hi−lo)/(hi+lo)   ← the force difference the two sides should have when fully locked (proportional to grip)
    #     Δ       = min(Δ_ideal, Δ_lock)    ← how much the differential can actually realize
    #     f_hi = total/2 + Δ/2 ,  f_lo = total/2 − Δ/2
    # ⚠ The first version wrote "when locked, directly f_hi = hi", which
    #   under small available force would give **all** the force to one
    #   side (f_lo reduced to zero) — caught on the spot by the
    #   symmetric-case assertion. The interpolation form satisfies both
    #   ends:
    #   symmetric case hi=lo ⇒ Δ_ideal=0 ⇒ strict 50/50 (all three
    #   differentials identical, no spurious differences);
    #   full load and locked, total=lo+hi ⇒ Δ=hi−lo ⇒ f_hi=hi, f_lo=lo
    #   (each reaching its own grip cap).
    denom = hi + lo
    delta = min(total * (hi - lo) / denom, d_lock) if denom > 0.0 else 0.0
    f_hi = total / 2.0 + delta / 2.0
    f_lo = total - f_hi
    return (f_lo, f_hi) if cap_l <= cap_r else (f_hi, f_lo)


def split_from_spec(cap_l: float, cap_r: float, f_avail: float, spec: dict,
                    radius_mm: float) -> tuple[float, float]:
    """从 API 传来的**力式**规格构造并分配（供 transient 的**历史路径**使用，见
    `transient.VehiclePlanar._diff_shares`：轮自由度关闭时走这条）。

    运行时真源是 `distribute_axle`（扭矩域，逐字移植前端）；本函数是力域准静态工具，
    两者的 spec 键不同（本函数读 kind/preload_nm/bias_ratio）。

    Build and allocate from the API's **force-style** spec (serves
    transient's **historical path**; see
    `transient.VehiclePlanar._diff_shares`: taken when the wheel DOF is
    off).

    The runtime source of truth is `distribute_axle` (torque domain,
    ported verbatim from the frontend); this function is the force-domain
    quasi-static tool. The two use different spec keys (this one reads
    kind/preload_nm/bias_ratio).
    """
    return axle_static_capacity_split(cap_l, cap_r, f_avail, Differential(
        kind=str(spec.get("kind", KIND_LOCKED)),
        preload_nm=float(spec.get("preload_nm", 0.0)),
        bias_ratio=float(spec.get("bias_ratio", 0.0)),
        radius_mm=radius_mm,
    ))


# ═══════════════════════════════════════════════════════════════════════════
# 运行时分配（真源）：逐字移植自 web/js/16-powertrain.js::distributeAxle
# ═══════════════════════════════════════════════════════════════════════════
# 参数映射（两内核口径对照，改动必须同步）：
#     diff["type"]        ↔ JS diff.type        ："open" | "locked" | "lsd"
#     diff["bias"]        ↔ JS diff.bias        扭矩偏置比 TBR ≥ 1
#     diff["lockNm"]      ↔ JS diff.lockNm      锁紧扭矩 N·m
#     diff["slipRefRadS"] ↔ JS diff.slipRefRadS 满锁滑差 rad/s
# 与 Differential 数据类的关系：Differential.bias_ratio = (TBR−1)/(TBR+1)、
# preload_nm ↔ lockNm —— 因形式同为 (TBR−1)/(TBR+1)，故映射是**恒等**的；
# 而力式 axle_static_capacity_split 用的是 bias_ratio/preload_nm（准静态工具，非运行时）。
#
# 语义（与前端逐条对齐，含失效安全）：
#     open/未知 → 严格 50/50（**不按载荷**；弱侧打滑时整轴上限自然被拖住）
#     locked    → 按垂直载荷分配；fz 和 ≤0 退化 50/50
#     lsd       → 从快轮向慢轮转移 Ttrans = min(lockNm, ((bias−1)/(bias+1))·|tAxle|/2 + ramp)
#                 ramp = lockNm·min(1, |Δω|/slipRef)；Δω = wR − wL，Δω≥0 → 向左轮转移
# 守恒：tR ≡ tAxle − tL（逐位），任意输入 |tL+tR−tAxle| < 1e−12。
# Runtime allocation (source of truth): ported verbatim from
# web/js/16-powertrain.js::distributeAxle
#
# Parameter mapping (comparison of the two kernels; changes must be
# synchronized on both sides):
#     diff["type"]        ↔ JS diff.type        : "open" | "locked" | "lsd"
#     diff["bias"]        ↔ JS diff.bias        torque bias ratio TBR ≥ 1
#     diff["lockNm"]      ↔ JS diff.lockNm      locking torque N·m
#     diff["slipRefRadS"] ↔ JS diff.slipRefRadS full-lock slip reference rad/s
# Relation to the Differential dataclass: Differential.bias_ratio =
# (TBR−1)/(TBR+1), preload_nm ↔ lockNm — since both forms are
# (TBR−1)/(TBR+1), the mapping is an **identity**; the force-form
# axle_static_capacity_split instead uses bias_ratio/preload_nm (a
# quasi-static tool, not the runtime path).
#
# Semantics (aligned item-by-item with the frontend, including fail-safe):
#     open/unknown → strict 50/50 (**not load-based**; when the weak side
#       spins, the axle cap is naturally dragged down by it)
#     locked    → allocate by vertical load; a fz sum ≤ 0 degrades to 50/50
#     lsd       → transfer from the fast wheel to the slow wheel:
#       Ttrans = min(lockNm, ((bias−1)/(bias+1))·|tAxle|/2 + ramp)
#                 ramp = lockNm·min(1, |Δω|/slipRef); Δω = wR − wL;
#                 Δω≥0 → transfer toward the left wheel
# Conservation: tR ≡ tAxle − tL (bit-exact); for any input,
# |tL+tR−tAxle| < 1e−12.


def distribute_axle(t_axle: float, w_l: float, w_r: float, fz_l: float, fz_r: float,
                    diff: dict | None) -> tuple[float, float]:
    """把一根轴的扭矩分到左右轮，返回 (tL, tR)（N·m/轮）。真源同前端 distributeAxle。

    Split one axle's torque between the left and right wheels; returns
    (tL, tR) (N·m per wheel). Same source as the frontend distributeAxle.
    """
    d = diff or {}
    ty = str(d.get("type", "open"))
    if ty == "locked":
        fl = float(fz_l or 0.0)
        fr = float(fz_r or 0.0)
        fs = fl + fr
        if fs > 0.0:
            t_l = t_axle * fl / fs
            return t_l, t_axle - t_l
        return t_axle / 2.0, t_axle / 2.0
    if ty == "lsd":
        bias = float(d["bias"]) if isinstance(d.get("bias"), (int, float)) and d["bias"] >= 1 else 1.0
        lock = float(d["lockNm"]) if isinstance(d.get("lockNm"), (int, float)) and d["lockNm"] > 0 else 0.0
        slip = (float(d["slipRefRadS"])
                if isinstance(d.get("slipRefRadS"), (int, float)) and d["slipRefRadS"] > 0 else 8.0)
        dw = float(w_r or 0.0) - float(w_l or 0.0)
        ramp = lock * min(1.0, abs(dw) / max(1e-3, slip))
        t_trans = min(lock, ((bias - 1.0) / (bias + 1.0)) * abs(t_axle) / 2.0 + ramp)
        tsign = 1.0 if dw >= 0.0 else -1.0
        t_l = t_axle / 2.0 + tsign * t_trans
        return t_l, t_axle - t_l
    return t_axle / 2.0, t_axle / 2.0
