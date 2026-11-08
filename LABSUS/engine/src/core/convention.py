"""坐标、符号与角度规范 — V1 冻结基线（设计文档 §12，P2-0 更新）。

本模块是 **V1 冻结线** 的内部真源（v3 主线的真源在 `src/api/v3models.py`
的 DWB_KEYS 与坐标注释，见 INTENTIONAL_DIFFERENCES.md「坐标系双血脉」）。
V1 冻结线当前仅被已弃用的 `src/core/models.py` 引用，无独立测试文件；
其条款由 tests/test_s1_gate.py 的手算对照与 tests/test_w4_consistency.py
的层间换算锚间接看守。修改约定必须同步本模块 + 设计文档 +
INTENTIONAL_DIFFERENCES.md。

坐标系
------
X：车辆前方（+X = 前）；Y：车辆右侧（+Y = 右）；Z：垂直向上。
原点：前轴中心地面。

角度符号（P2-0 冻结，左右两侧统一车辆全局约定）
------------------------------------------------
- Camber：负 = 内倾（胎顶朝车辆中心线），两侧相同。
  公式：camber = -atan2(spin_z, hypot(spin_x, spin_y))。
- Toe：正 = Toe-in（轮前指向车辆中心线），两侧相同。
  公式：toe = -side · atan2(fwd_y, fwd_x)，fwd 为轮面前向水平迹线（+X 方向）。
  side = +1 右轮 / -1 左轮（按轮心 Y 判别）。注：平行转向时两侧 toe 异号
  （内轮 toe-in、外轮 toe-out），这是物理正确行为，不再做显示对齐。
- KPI/SAI：正 = 主销轴上端向车内倾斜，两侧相同。
  公式：kpi = side · atan2(kp_vec[1], -kp_vec[2])。
- Caster：正 = 主销轴上端后倾（经典定义），两侧相同。
  公式：caster = atan2(kp_vec[0], -kp_vec[2])。
- Scrub：正 = 接地印迹在主销接地点外侧（正拖距/正主销偏距），两侧相同。
  公式：scrub = side · (contact_y - kingpin_ground_y)。
- Trail：正 = 主销接地点在接地印迹前方（经典机械拖距），两侧相同。
  公式：caster_trail_mm = kingpin_ground_x - contact_x。

禁止以 fix_left_angles、toe<90° 等后处理特判作为规范（设计文档 §12）。
compute_alignment_angles 已按侧别直接输出车辆全局约定，无需翻号。

转向
----
- rack_displacement 正 = 齿条向车辆右侧（+Y）平移。
- 实测：rack=+5mm → 右轮 toe ≈ +3.3°（toe-in），左轮 toe ≈ -3.3°（toe-out），
  两轮前向同指 -Y，为平行同向偏转。

已知规范问题登记（Known Issues）
--------------------------------
- K-1（P2-0 已修复）：左右镜像 Scrub 不对称，根因是 compute_contact_patch 的
  旧坐标系 "X=up" 外倾公式与同向横向偏移；已改为轮面投影，镜像对称。
- K-2（P2-0 已修复）：toe 符号已统一为 toe-in 为正，并写入方案元数据。
- K-3（P2-0 已修复）：caster 公式与 derive_hardpoints 主销倾角符号已统一为
  经典定义（正 = 轴上端后倾）；caster_trail_mm 即经典机械拖距。
- K-4（已关闭，2026-08-22 起）：旧"顺序解 + polish 分支锚定"在负行程侧残差超标
  （-10mm ≈ 0.10mm、-15mm ≈ 0.19mm），未达 §13.2 全区间 ≤0.02mm 目标。该实现已
  整体退役——现行 `src/solver/mechanism/solver.py` 改用 scipy least_squares(TRF)
  联合收敛，实测（2026-09-20）±80mm 全行程最差 1.14e-13mm、0 点超 0.02mm 门禁，
  由 tests/test_s1_gate.py G1 看守。本条仅作历史记录，勿据此判断现行求解器精度。

Coordinates, signs, and angle conventions — the V1 frozen baseline
(design doc §12, updated at P2-0).

This module is the internal source of truth for the **V1 frozen line**
(the source of truth for the v3 mainline lives in the DWB_KEYS and
coordinate comments of `src/api/v3models.py`; see
INTENTIONAL_DIFFERENCES.md "two coordinate lineages"). The V1 frozen line
is currently referenced only by the deprecated `src/core/models.py` and
has no dedicated test file; its clauses are indirectly guarded by the
hand-calculation comparisons in tests/test_s1_gate.py and the
between-layer conversion anchors in tests/test_w4_consistency.py. Any
change to the conventions must update this module + the design doc +
INTENTIONAL_DIFFERENCES.md together.

Coordinate system
-----------------
X: vehicle forward (+X = front); Y: vehicle right (+Y = right); Z:
vertically up. Origin: ground, front-axle center.

Angle signs (frozen at P2-0; one vehicle-global convention for both sides)
------------------------------------------------
- Camber: negative = tilted inward (tire top toward the vehicle
  centerline), same on both sides.
  Formula: camber = -atan2(spin_z, hypot(spin_x, spin_y)).
- Toe: positive = toe-in (the wheel's front points toward the vehicle
  centerline), same on both sides.
  Formula: toe = -side · atan2(fwd_y, fwd_x), where fwd is the wheel
  plane's forward horizontal trace (+X direction).
  side = +1 right wheel / -1 left wheel (decided by the wheel-center Y).
  Note: with parallel steering the two sides' toes have opposite signs
  (inner wheel toe-in, outer wheel toe-out); this is physically correct
  behavior and no display alignment is applied.
- KPI/SAI: positive = the kingpin axis top tilts inboard, same on both
  sides.
  Formula: kpi = side · atan2(kp_vec[1], -kp_vec[2]).
- Caster: positive = the kingpin axis top leans rearward (classic
  definition), same on both sides.
  Formula: caster = atan2(kp_vec[0], -kp_vec[2]).
- Scrub: positive = the contact patch is outboard of the kingpin ground
  point (positive trail / positive scrub radius), same on both sides.
  Formula: scrub = side · (contact_y - kingpin_ground_y).
- Trail: positive = the kingpin ground point is ahead of the contact
  patch (classic mechanical trail), same on both sides.
  Formula: caster_trail_mm = kingpin_ground_x - contact_x.

Post-hoc special cases such as fix_left_angles or toe<90° are forbidden
as conventions (design doc §12).
compute_alignment_angles already outputs the vehicle-global convention
directly per side; no sign flipping is needed.

Steering
--------
- rack_displacement positive = the rack translates toward the vehicle's
  right (+Y).
- Measured: rack=+5mm → right wheel toe ≈ +3.3° (toe-in), left wheel toe
  ≈ -3.3° (toe-out); both wheels' forward traces point toward -Y, i.e. a
  parallel same-direction deflection.

Registered known convention issues (Known Issues)
--------------------------------
- K-1 (fixed at P2-0): left/right mirrored Scrub was asymmetric; the
  root cause was compute_contact_patch's old "X=up" frame camber formula
  plus the same-direction lateral offset; replaced by the wheel-plane
  projection, mirror-symmetric.
- K-2 (fixed at P2-0): the toe sign was unified to toe-in positive and
  written into the design metadata.
- K-3 (fixed at P2-0): the caster formula and the derive_hardpoints
  kingpin inclination sign were unified to the classic definition
  (positive = axis top rearward); caster_trail_mm is the classic
  mechanical trail.
- K-4 (closed, since 2026-08-22): the old "sequential solve + polish
  branch anchoring" exceeded the residual budget on the negative-travel
  side (-10mm ≈ 0.10mm, -15mm ≈ 0.19mm), missing the §13.2 target of
  ≤0.02mm across the full range. That implementation was retired
  wholesale — the current `src/solver/mechanism/solver.py` instead uses
  scipy least_squares(TRF) joint convergence; measured (2026-09-20):
  worst 1.14e-13mm over the full ±80mm travel, and the 0 point passes
  the 0.02mm gate, guarded by tests/test_s1_gate.py G1. This entry is
  kept for history only; do not judge the current solver's accuracy by
  it.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class CoordinateConvention:
    """嵌入每个 ChassisDesign 元数据，声明该方案数值所遵循的坐标与符号约定。

    Embedded into every ChassisDesign's metadata, declaring the
    coordinate and sign conventions its numbers follow.
    """

    x_axis: str = "forward"
    y_axis: str = "right"
    z_axis: str = "up"
    origin: str = "front_axle_center_ground"
    camber_positive: str = "top_out"                   # 正 = 顶端外倾（负 = 内倾） — positive = top tilts outward (negative = leans inward)
    toe_positive: str = "toe_in_positive"              # 正 = Toe-in（P2-0） — positive = toe-in (P2-0)
    kpi_positive: str = "axis_top_inboard"
    caster_positive: str = "axis_top_rearward"         # 经典（P2-0） — classic (P2-0)
    scrub_definition: str = "contact_outboard_of_kingpin_positive"  # P2-0
    trail_definition: str = "kingpin_ground_ahead_of_contact_positive"  # 经典（P2-0） — classic (P2-0)
    rack_positive: str = "rack_toward_vehicle_right"
    spec_revision: str = "v1-p2-0"


DEFAULT_CONVENTION = CoordinateConvention()

# 已知规范问题登记真源在模块 docstring（K-1..K-4，全部已关闭）。
# 旧版此处有 KNOWN_ISSUES=("K-4",) 常量：无任何消费方，且 K-4 已关闭，
# 保留会误导读者以为仍有未关闭项 —— 2026-09-20 删除。
# The source of truth for the known-issues register is the module
# docstring (K-1..K-4, all closed). An old version had a
# KNOWN_ISSUES=("K-4",) constant here: it had no consumers and K-4 was
# closed; keeping it would mislead readers into thinking items remain
# open — deleted 2026-09-20.
