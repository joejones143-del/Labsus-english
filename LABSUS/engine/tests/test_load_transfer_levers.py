"""载荷转移杠杆有效性守卫（M8，2026-09-20）。

背景（本轮实证修复的物理错误）：载荷敏感性 LS 的**参考载荷**被写成了 FzNom=3500 N
（GT3 量级），而 FSAE 实际轮载仅约 580 N。于是

    dμ/dFz = −LS·Fy0/FzNom²

被稀释约 6 倍：实测前 ARB 直径 0→30 mm 时前轴滚转刚度 kphi_f 从 470 涨到
3351 N·m/°、TLLTD 45.5%→74.9%、侧倾梯度 0.456→0.119 °/g，**而 US 梯度只从
0.2104 动到 0.2262（+0.016 °/g）**——载荷转移类杠杆（ARB / 弹簧刚度 / 配重）
在模型里形同虚设，用户调 ARB 看不到操稳变化。

修复：把"载荷敏感性参考"与"μ 量级参考"分离（`tire_mf.load_d(FzRef=...)`），
参考取整车静轮载均值 mT·g/4（两内核 + 瞬态层同一个数）。修后同一处杠杆
实测 +0.083 °/g（约 5.2 倍）。

本文件钉住**结论性质**而不是具体数值：
  1) 前 ARB 由拆除到 30 mm，US 梯度的变化量必须 ≥ 0.03 °/g（旧代码实测 0.016 ⇒ FAIL）；
  2) 方向正确：前轴更硬 ⇒ 不足转向增加（US 单调不减）；
  3) 前弹簧刚度是有效杠杆（≥ 0.02 °/g，旧代码 0.010 ⇒ FAIL）；
  4) FzRef 真的被消费（显式给出与缺省结果必须不同）。

English: load-transfer lever effectiveness guard (M8, 2026-09-20).
Background (physical error fixed with evidence this round): the load-sensitivity LS reference load was written as FzNom=3500 N
(GT3 scale) while the real FSAE wheel load is only about 580 N. So

    dμ/dFz = −LS·Fy0/FzNom²

was diluted about 6x: with the front ARB diameter moved 0→30 mm, measured front-axle roll stiffness kphi_f rose from 470 to
3351 N·m/°, TLLTD 45.5%→74.9%, roll gradient 0.456→0.119 °/g — while the US gradient moved only from 0.2104 to 0.2262
(+0.016 °/g) — the load-transfer levers (ARB / spring rate / ballast) were effectively decorative in the model, and the user
saw no handling change when tuning the ARB.

Fix: separate the load-sensitivity reference from the μ magnitude reference (`tire_mf.load_d(FzRef=...)`); the reference is the
mean static wheel load mT·g/4 (the same number in both kernels + the transient layer). After the fix the same lever measures
+0.083 °/g (about 5.2x).

This file pins qualitative conclusions, not specific numbers:
  1) front ARB removed→30 mm must change the US gradient by ≥ 0.03 °/g (old code measured 0.016 ⇒ FAIL);
  2) the direction is correct: a stiffer front ⇒ more understeer (US monotonically non-decreasing);
  3) the front spring rate is an effective lever (≥ 0.02 °/g, old code 0.010 ⇒ FAIL);
  4) FzRef is actually consumed (explicit value vs default results must differ).
"""
from src.api.chassis import solve_chassis
from src.api.v3models import ArbSpec, ChassisRequest, QuasiInputs, TireParams, VehicleSpec
from src.api.v3service import DEFAULT_DWB_POINTS
from src.tire_mf import load_d

GY = 1.65


def _vehicle(arb_d: float = 0.0, spring_rate: float = 105.0,
             camber_gain: float = 0.0, camber_f: float = -1.5,
             camber_r: float = -1.75) -> VehicleSpec:
    """接近 formula 预设的轻量车：mS 185 / mU 12·14 / wb 1550 / 静轮载约 580 N。"""  # English: a lightweight car near the formula preset: mS 185 / mU 12·14 / wb 1550 / static wheel load ≈580 N.
    return VehicleSpec(**{
        "wheelbase_mm": 1550.0, "mass_kg": 235.0, "sprung_mass_kg": 185.0,
        "hcg_mm": 270.0, "hs_mm": 285.0,
        "front": {"points": DEFAULT_DWB_POINTS, "arch": "pushrod",
                  "camber_deg": camber_f, "toe_deg": -0.05, "tire_radius": 250.0,
                  "camber_gain_deg_per_25mm": camber_gain,
                  "spring_rate": spring_rate, "spring_mass_kg": 92.5,
                  "unsprung_kg": 12.0, "motion_ratio": 0.6,
                  "arb": ArbSpec(d=arb_d)},
        "rear": {"points": DEFAULT_DWB_POINTS, "arch": "pullrod",
                 "camber_deg": camber_r, "toe_deg": 0.2, "tire_radius": 260.0,
                 "camber_gain_deg_per_25mm": camber_gain,
                 "spring_rate": 125.0, "spring_mass_kg": 92.5,
                 "unsprung_kg": 14.0, "motion_ratio": 0.7,
                 "arb": ArbSpec(d=0.0)},
    })


def _us_grad(vehicle: VehicleSpec, tire: TireParams | None = None) -> float:
    req = ChassisRequest(vehicle=vehicle, quasi=QuasiInputs(gy=GY),
                         tire=tire if tire is not None else TireParams())
    return solve_chassis(req).loads.us_grad_deg_per_g


def test_front_arb_is_an_effective_balance_lever():
    """前 ARB 拆除 → 30 mm，US 梯度变化必须可感（阈值 0.03 °/g）。

    旧实现（LS 相对 FzNom）实测仅 0.016 °/g ⇒ 本断言 FAIL；修复后 0.083。

    English: front ARB removed→30 mm must produce a perceptible US-gradient change (threshold 0.03 °/g).
    The old implementation (LS relative to FzNom) measured only 0.016 °/g ⇒ this assertion FAILed; after the fix, 0.083.
    """
    soft = _us_grad(_vehicle(arb_d=0.0))
    stiff = _us_grad(_vehicle(arb_d=30.0))
    delta = stiff - soft
    assert delta >= 0.03, (
        f"前 ARB 0→30mm 只让 US 变 {delta:+.4f} °/g（0.2104→0.2262 为病灶态）"
        f"：载荷转移杠杆失效，检查 tire_mf.load_d 的 FzRef 是否被消费")
    assert delta > 0, "前轴更硬应增加不足转向"


def test_front_arb_monotonic_in_balance():
    """杠杆方向单调：ARB 直径递增 ⇒ US 不减（软→中→硬）。"""  # English: the lever direction is monotonic: increasing ARB diameter ⇒ US never decreasing (soft→medium→hard).
    us = [_us_grad(_vehicle(arb_d=d)) for d in (0.0, 12.0, 20.0, 30.0)]
    assert all(b >= a - 1e-9 for a, b in zip(us, us[1:])), us


def test_front_spring_rate_is_an_effective_lever():
    """弹簧刚度同样是载荷转移杠杆（阈值 0.02 °/g；旧实现 0.010 ⇒ FAIL）。"""  # English: the spring rate is also a load-transfer lever (threshold 0.02 °/g; old implementation 0.010 ⇒ FAIL).
    soft = _us_grad(_vehicle(spring_rate=105.0))
    stiff = _us_grad(_vehicle(spring_rate=300.0))
    assert stiff - soft >= 0.02, f"前弹簧 105→300 N/mm 只让 US 变 {stiff - soft:+.4f} °/g"


def test_explicit_fzref_is_consumed():
    """显式 FzRef 必须真的改变结果，且**杠杆倍数**必须显著放大。

    这是本次修复的定量主张：同一台车、同一个 ARB 变化，
    参考载荷=静轮载时的 US 变化量应 ≥ 参考载荷=FzNom（旧稀释口径）的 3 倍
    （实测约 5 倍）。旧代码下两者恒等 ⇒ 本断言 FAIL。

    English: an explicit FzRef must actually change the result, and the leverage ratio must be significantly amplified.
    This is the quantitative claim of this fix: same car, same ARB change — with the reference load at the static wheel load, the
    US change should be ≥ 3x the FzNom-reference (old diluted) case, measured about 5x. Under the old code the two were identical ⇒ this assertion FAILed.
    """
    def lever(ref: float | None) -> float:
        t = TireParams(FzRef=ref)
        return _us_grad(_vehicle(arb_d=30.0), t) - _us_grad(_vehicle(arb_d=0.0), t)

    d_static = lever(578.0)
    d_nominal = lever(3500.0)
    d_default = lever(None)
    assert abs(d_static - d_nominal) > 1e-4, "FzRef 未被消费"
    assert d_static >= 3.0 * d_nominal, (
        f"静载参考的杠杆 {d_static:+.4f} °/g 未显著大于 FzNom 口径 {d_nominal:+.4f} °/g"
        f"——载荷转移杠杆仍被稀释")
    # 缺省（引擎自导静轮载均值）应落在同一量级，而不是退回 FzNom 口径 — the default (engine-derived static-wheel-load mean) should land in the same magnitude, not fall back to the FzNom convention
    assert d_default >= 2.5 * d_nominal, (d_default, d_nominal)


def test_fzref_semantics_at_reference():
    """纯函数语义：FzRef 处无修正；载荷翻倍 μ 恰降 LS。"""  # English: pure-function semantics: no correction at FzRef; doubling the load lowers μ by exactly LS.
    ref, ls, fy0, fzn = 578.0, 0.10, 5250.0, 3500.0
    d_ref = load_d(ref, Fy0=fy0, FzNom=fzn, LS=ls, FzRef=ref)
    d_2x = load_d(2 * ref, Fy0=fy0, FzNom=fzn, LS=ls, FzRef=ref)
    assert abs(d_ref - fy0 * ref / fzn) < 1e-9
    assert abs((d_2x / (2 * ref)) / (d_ref / ref) - (1 - ls)) < 1e-12


# ── M11（2026-09-20）· P12 外倾→抓地耦合守卫 ──────────────────────────────── — ( M11 (2026-09-20) · P12 camber→grip coupling guard )
def test_camber_coupling_is_switchable_and_changes_us():
    """外倾耦合必须可关可开，且开启后显著改变 US 梯度。

    缺省 `camber_gain_deg_per_25mm = 0` ⇒ 耦合项恒 0、历史行为逐位不变（直接调 API
    的调用方不受影响）；给出真实外倾增益（前端把扫掠得到的 cg 下发）后，静态外倾
    才第一次能影响平衡——此前模型对 camber 完全盲。

    English: the camber coupling must be switchable off and on, and once on it must change the US gradient significantly.
    Default `camber_gain_deg_per_25mm = 0` ⇒ the coupling term stays 0 and historical behavior remains bit-identical (direct-API
    callers unaffected); only once a real camber gain is given (the frontend sends down the swept cg) can static camber influence
    balance for the first time — previously the model was completely blind to camber.
    """
    off = _us_grad(_vehicle(camber_gain=0.0))
    on = _us_grad(_vehicle(camber_gain=-1.0))          # formula 实测 cg ≈ -1.0 °/25mm — formula measured cg ≈ -1.0 °/25mm
    assert abs(on - off) > 0.02, (
        f"开启外倾耦合后 US 只变 {on - off:+.4f} °/g（实测约 -0.08）——耦合未生效")


def test_static_camber_only_matters_when_coupled():
    """静态外倾对 US 的影响只应来自耦合项：关掉耦合 ⇒ 改外倾完全不改变 US。"""  # English: the static camber effect on US should come only from the coupling term: coupling off ⇒ changing camber leaves US completely unchanged.
    off_a = _us_grad(_vehicle(camber_gain=0.0, camber_f=-1.0))
    off_b = _us_grad(_vehicle(camber_gain=0.0, camber_f=-3.0))
    # 耦合关闭时静态外倾只应剩二阶残余：实测 ~1e-4（0.5%），来自 chassis 内部
    # 侧倾中心/外倾增益扫掠的缓存键含 camber_deg（该扫掠结果间接进入载荷转移）。
    # 与"耦合开启后 ≥0.02"相比小两个数量级，故门限取 1e-3 并把量级写在这里。
    # with the coupling off, static camber should leave only a second-order residual: measured ~1e-4 (0.5%), coming from
    # the chassis-internal roll-center/camber-gain sweep whose cache key includes camber_deg (that sweep result enters
    # load transfer indirectly). Two orders of magnitude smaller than the ≥0.02 with coupling on, so the threshold is 1e-3
    # and the magnitude is written down here.
    assert abs(off_b - off_a) < 1e-3, (
        f"耦合关闭时静态外倾仍让 US 变 {off_b - off_a:+.5f} °/g（应仅剩 1e-4 级二阶残余）")

    on_a = _us_grad(_vehicle(camber_gain=-1.0, camber_f=-1.0))
    on_b = _us_grad(_vehicle(camber_gain=-1.0, camber_f=-3.0))
    assert abs(on_b - on_a) > 0.02, (
        f"耦合开启后改前外倾 2° 只让 US 变 {on_b - on_a:+.4f} °/g——通道没接通")
    # 方向：前轴外倾更负 ⇒ 前轴外倾推力更大 ⇒ 前轴所需侧偏角更小 ⇒ US 减小 — direction: more negative front camber ⇒ larger front camber thrust ⇒ smaller required front slip angle ⇒ US decreases
    assert on_b < on_a, "前轴外倾加深应减少不足转向（外倾推力帮忙）"
