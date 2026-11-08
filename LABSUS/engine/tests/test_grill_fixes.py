"""grill-me 修复波次回归测试（2026-08-30，报告 F-09/F-15/F-16/F-22/F-24/F-25/F-63）。

断言物理性质而非魔法数（第 11 讲方法论）：
- TLLTD 符号对称性（F-09：负 g 不再恒 50%）
- 侧倾中心 0 值不被回退吞掉（F-15）
- 侧倾失稳显式上报（F-16）
- 松弛解析解在极端步长下不振荡发散（F-24）
- 响应 JSON 严格合法（F-25：无 NaN/Infinity 字面量）
- 洗刷函数边界行为（F-25）
- compliance 锚点映射失败显式抛错（F-63）

English: grill-me fix-wave regression tests (2026-08-30, report F-09/F-15/F-16/F-22/F-24/F-25/F-63).
Assert physical properties rather than magic numbers (Lecture-11 methodology):
- TLLTD sign symmetry (F-09: negative g no longer pinned at 50%)
- legitimate roll-center 0 values are not swallowed by the fallback (F-15)
- roll instability is explicitly reported (F-16)
- the analytic relaxation solution does not oscillate or diverge at extreme step sizes (F-24)
- response JSON strictly valid (F-25: no NaN/Infinity literals)
- wash-function boundary behavior (F-25)
- compliance anchor-mapping failure raises explicitly (F-63)
"""
import json
import math

import numpy as np
from fastapi.testclient import TestClient
from server import app  # noqa: E402

from src.api.chassis import quasi_loads  # noqa: E402
from src.api.v3models import QuasiInputs, VehicleSpec  # noqa: E402
from src.solver.transient import _wash_json  # noqa: E402

client = TestClient(app)

FRONT = {
    "LCA_F": [260, 140, 130], "LCA_R": [260, -120, 140], "UCA_F": [350, 110, 340],
    "UCA_R": [350, -100, 350], "LBJ": [640, 10, 150], "UBJ": [590, -15, 410],
    "WC": [710, 0, 280], "TRO": [600, -120, 185], "RACK": [220, -135, 180],
    "STRUT_OUT": [595, 10, 185], "RCK_AX_A": [300, 15, 330], "RCK_AX_B": [300, 70, 328],
    "STRUT_IN": [315, 45, 400], "RCK_DMP": [230, 48, 345], "DMP_BODY": [30, 48, 170],
}
REAR = {
    "LCA_F": [250, 140, 115], "LCA_R": [250, -130, 125], "UCA_F": [340, 110, 320],
    "UCA_R": [340, -110, 330], "LBJ": [630, 10, 135], "UBJ": [580, -15, 390],
    "WC": [700, 0, 290], "TRO": [590, -120, 170], "RACK": [210, -130, 160],
    "STRUT_OUT": [570, -15, 380], "RCK_AX_A": [340, 25, 290], "RCK_AX_B": [340, 75, 292],
    "STRUT_IN": [370, 45, 255], "RCK_DMP": [305, 60, 270], "DMP_BODY": [25, 62, 310],
}


def _vehicle():
    return {
        "wheelbase_mm": 1620.0, "mass_kg": 480.0, "sprung_mass_kg": 420.0,
        "hcg_mm": 320.0, "hs_mm": 330.0,
        "front": {"points": FRONT, "arch": "pushrod", "camber_deg": -1.2,
                  "toe_deg": 0.05, "tire_radius": 305.0, "spring_rate": 110.0,
                  "spring_mass_kg": 300.0, "unsprung_kg": 38.0,
                  "motion_ratio": 0.75},
        "rear": {"points": REAR, "arch": "pullrod", "camber_deg": -1.5,
                 "toe_deg": 0.1, "tire_radius": 310.0, "spring_rate": 130.0,
                 "spring_mass_kg": 340.0, "unsprung_kg": 42.0,
                 "motion_ratio": 0.78},
    }


def _req(**over):
    body = {
        "vehicle": _vehicle(),
        "quasi": {"gy": 0.0, "gx": 0.0, "aero_force_n": 0.0, "aero_bias": 0.5},
        "travel": {"fl": 0.0, "fr": 0.0, "rl": 0.0, "rr": 0.0},
        "rack": 0.0,
    }
    body.update(over)
    return body


# ── F-09：TLLTD 符号对称性 ───────────────────────────────────── — ( F-09: TLLTD sign symmetry )

def test_tlltd_negative_g_not_pinned_at_50():
    """左转（gy<0）时 TLLTD 必须镜像而非恒 50%；右转/左转前轴份额对称相等。

    物理：TLLTD = dFz_f_tot / (dFz_f_tot + dFz_r_tot)。gy 变号时分子分母
    同号翻转，比值不变（左右镜像对称车）→ pos ≈ neg（并非互补 100）。
    F-09 修复目标：gy<0 不再因 `sum_transfer > 1` 守卫误判而恒回退 50%。

    English: when turning left (gy<0) TLLTD must mirror rather than stay pinned at 50%; the front-axle share is symmetrically equal for right/left turns.
    Physics: TLLTD = dFz_f_tot / (dFz_f_tot + dFz_r_tot); when gy flips sign, numerator and denominator flip sign together and
    the ratio stays unchanged (left/right mirror-symmetric car) → pos ≈ neg (not complementary 100). F-09 fix goal: gy<0 no longer
    falls back to 50% permanently through the misjudging `sum_transfer > 1` guard.
    """
    pos = client.post("/api/v3/chassis/solve",
                      json=_req(quasi={"gy": 1.0, "gx": 0.0, "aero_force_n": 0.0,
                                       "aero_bias": 0.5})).json()["loads"]["tlltd_pct"]
    neg = client.post("/api/v3/chassis/solve",
                      json=_req(quasi={"gy": -1.0, "gx": 0.0, "aero_force_n": 0.0,
                                       "aero_bias": 0.5})).json()["loads"]["tlltd_pct"]
    assert pos != 50.0 and neg != 50.0, (pos, neg)
    assert abs(pos - neg) < 1.0, (pos, neg)   # 镜像对称：左右转前轴份额相等 — mirror symmetry: front-axle share equal for left/right turns
    assert 0.0 < pos < 100.0 and 0.0 < neg < 100.0, (pos, neg)


# ── F-15：合法 0 值不被回退吞掉 ──────────────────────────────── — ( F-15: legitimate 0 values are not swallowed by the fallback )

def _zero_sweep():
    """合法最小 rc_sw：travel/rc_h 键齐备；恒定 0 —— 0 行程迁移保持 0。

    注意：dFz_geo 取迭代**迁移后**的 zrc_f（sweep 插值），故 sweep 必须
    恒定 0 才能让"rc_h=0 恰在地面"场景的几何传力路径精确为 0；
    两侧非零的 sweep 会因侧倾迁移（dt_f≠0）引入非零 geo，误伤断言。

    English: a legitimate minimal rc_sw: travel/rc_h keys present; constant 0 — zero-travel migration stays 0.
    Note: dFz_geo takes the post-migration zrc_f of the iteration (sweep interpolation), so the sweep must be constant 0 for the
    geometric load-transfer path in the "rc_h=0 exactly at ground" scenario to be exactly 0; a non-zero sweep would introduce
    non-zero geo through roll migration (dt_f≠0) and unfairly kill the assertion.
    """
    return {"front": {"travel": [-90.0, -60.0, 0.0, 60.0, 90.0],
                      "rc_h": [0.0, 0.0, 0.0, 0.0, 0.0]},
            "rear": {"travel": [-90.0, -60.0, 0.0, 60.0, 90.0],
                     "rc_h": [0.0, 0.0, 0.0, 0.0, 0.0]}}


def test_quasi_loads_zero_rc_height_not_swallowed():
    """rc_h=0（侧倾中心恰在地面）时几何传力路径必须为 0，而非按 45mm 兜底计算。"""  # English: with rc_h=0 (roll center exactly at ground) the geometric load-transfer path must be 0, not computed with the 45mm fallback.
    v = VehicleSpec(**_vehicle())
    q = QuasiInputs(gy=1.0, gx=0.0, aero_force_n=0.0, aero_bias=0.5)
    loads = quasi_loads(v, q, {"front": 0.75, "rear": 0.78},
                        {"FR": 0.0, "RR": 0.0}, _zero_sweep())
    assert loads.dFz_geo_f == 0.0
    assert loads.dFz_geo_r == 0.0


# ── F-16：侧倾失稳显式上报 ───────────────────────────────────── — ( F-16: roll instability explicitly reported )

def test_quasi_loads_roll_instability_reports_warning():
    """kphi_tot 被削到极小 → denom ≤ 100 → 侧倾置零 + warning 显式上报。

    注意：必须显式拆掉 ARB（arb.d=0），否则默认 d=18 的扭杆刚度仍能
    支撑 kphi_tot > mS·g·h_arm，弹簧归零也触发不了失稳判据。

    English: kphi_tot shaved to nearly nothing → denom ≤ 100 → roll zeroed + a warning explicitly reported.
    Note: the ARB must be explicitly removed (arb.d=0), otherwise the default d=18 torsion-bar stiffness alone can support
    kphi_tot > mS·g·h_arm, and zeroing the springs would never trigger the instability criterion.
    """
    v = VehicleSpec(**_vehicle())
    v.front.spring_rate = 0.01          # 侧倾刚度近乎归零 → 触发失稳判据 — roll stiffness nearly zeroed → triggers the instability criterion
    v.rear.spring_rate = 0.01
    v.front.arb.d = 0.0                 # 拆 ARB（d<=0.5 视为拆除） — remove the ARB (d<=0.5 counts as removed)
    v.rear.arb.d = 0.0
    warns: list[str] = []
    q = QuasiInputs(gy=1.0, gx=0.0, aero_force_n=0.0, aero_bias=0.5)
    loads = quasi_loads(v, q, {"front": 0.75, "rear": 0.78},
                        {"FR": 50.0, "RR": 50.0}, _zero_sweep(),
                        warnings=warns)
    assert loads.roll_deg == 0.0
    assert any("roll divergence" in w for w in warns), warns  # 引擎告警原文: roll divergence... — engine warning text: roll divergence...


# ── F-24：解析松弛在极端步长下稳定 ───────────────────────────── — ( F-24: the analytic relaxation stays stable at extreme step sizes )

def test_relaxation_stable_at_extreme_dt():
    """dt=0.05 / vx=40 / σ=0.3（旧显式欧拉比率 6.7 → 发散）下仿真仍须收敛稳定。

    G24（2026-09-02）：半径从 30m 改为 150m。原工况 r=30m @40m/s 要求
    a_y = v²/R = 1600/30 = 53.3 m/s² = **5.44g**，任何轮胎都做不到（旧缺省胎
    μ=2.29 也只能 2.29g）——车根本跑不上这条线，一直在深度饱和里跑偏，
    |alphaL|<60° 那个断言只是在“能不能不爆”的边缘跑运气。它当时能过，
    恰恰是因为 μ=2.29 这个不真实的占位值把车“托”住了。
    现取 R=150m ⇒ a_y = 1600/150 = 10.7 m/s² = 1.09g，在 μ=1.50 下可达；
    而数值应力本意（dt·vx/σ = 0.05×40/0.3 = 6.7 ≫ 2）完整保留。

    English: with dt=0.05 / vx=40 / σ=0.3 (old explicit-Euler ratio 6.7 → divergence) the simulation must still converge stably.
    G24 (2026-09-02): the radius changed from 30m to 150m. The old case r=30m @40m/s demanded a_y = v²/R = 1600/30 = 53.3 m/s² =
    5.44g, which no tire can deliver (the old default tire μ=2.29 could deliver only 2.29g) — the car simply could not run that
    line, it stayed permanently off-line in deep saturation, and the |alphaL|<60° assertion was just gambling on survival. It
    passed only because the unrealistic placeholder μ=2.29 propped the car up. Now R=150m ⇒ a_y = 1600/150 = 10.7 m/s² = 1.09g,
    achievable at μ=1.50, while the intended numerical stress (dt·vx/σ = 0.05×40/0.3 = 6.7 ≫ 2) is fully retained."""
    track = [{"x": R * math.cos(th), "y": R * math.sin(th), "target_speed": 40.0}
             for R in [150.0] for th in np.linspace(0, 2 * math.pi, 33)][:-1]
    body = {"vehicle": _vehicle(), "track": track, "dt": 0.05,
            "sim_time": 6.0, "start_speed": 40.0}
    r = client.post("/api/v3/chassis/simulate_track", json=body)
    assert r.status_code == 200
    b = r.json()
    assert b["status"] == "VALID", b.get("warnings")
    # 动作已物理可行：|alphaL| 应落在真实胎的工作区间（峰值 6~10°，留饱和余量） — the maneuver is now physically feasible: |alphaL| should land inside the real-tire operating band (peak 6–10°, saturation headroom left)
    worst = max(max(abs(p[f"alphaL_{w}"]) for w in ("FR", "FL", "RR", "RL"))
                for p in b["trace"])
    assert worst < 25.0, worst
    # 速度未失控（本测试只守“松弛积分不发散”，不守循迹质量） — speed did not run away (this test guards only that the relaxation integration does not diverge, not path-tracking quality)
    assert max(abs(p["vx"]) for p in b["trace"]) < 60.0
    # ※ 待查（不属本测试契约，故不在此断言）：R=150m @40m/s 下 max|r|=0.79 rad/s，
    #   而 v/R=0.267。DriverPI 纯追踪在“大半径 + 前视 18m”工况下有过转瞬态
    #   （疑似 phantom bracket）。旧工况 R=30m 从未暴露它，因为车根本跑不上那条线。
    #   归入 G24 待办，与前端赛道舞台的循迹质量一起查。
    # open questions (not part of this test contract, so not asserted here): at R=150m @40m/s max|r|=0.79 rad/s while
    # v/R=0.267. DriverPI pure pursuit showed a brief yaw transient in the large-radius + 18m-lookahead case (suspected
    # phantom bracket). The old R=30m case never exposed it because the car could not run that line at all. Filed under
    # the G24 backlog, to be investigated together with the path-tracking quality of the frontend track stage.


# ── F-25：响应 JSON 严格合法 ─────────────────────────────────── — ( F-25: response JSON strictly valid )

def test_track_response_strict_json_no_nan_literals():
    track = [{"x": 0, "y": 0, "target_speed": 15.0},
             {"x": 80, "y": 0, "target_speed": 15.0}]
    r = client.post("/api/v3/chassis/simulate_track",
                    json={"vehicle": _vehicle(), "track": track,
                          "dt": 0.01, "sim_time": 5.0, "start_speed": 8.0})
    assert r.status_code == 200
    for token in ("NaN", "Infinity", "-Infinity"):
        assert token not in r.text, token


def test_wash_json_boundaries():
    assert _wash_json(float("nan")) is None
    assert _wash_json(float("inf")) is None
    assert _wash_json(1.5) == 1.5
    assert _wash_json({"a": [float("nan"), 2]}) == {"a": [None, 2]}
    assert _wash_json("NaN") == "NaN"            # 字符串不动 — strings unchanged
    # 洗刷后的 dict 必须能通过严格 JSON 序列化 — the washed dict must survive strict JSON serialization
    json.dumps(_wash_json({"x": float("nan"), "y": [1, float("inf")]}),
               allow_nan=False)


# ── F-63：compliance 锚点映射失败必须显式报错 ────────────────── — ( F-63: compliance anchor-mapping failure must raise explicitly )

def test_compliance_unmapped_anchor_raises():
    from src.api.v3service import DEFAULT_DWB_POINTS, make_bushings, to_engine_points
    from src.solver.compliance import solve_compliance_full
    from src.solver.forces import LinkForce, QSLoad
    from src.solver.mechanism.models import build_mechanism

    eng = to_engine_points(DEFAULT_DWB_POINTS)
    mech = build_mechanism(eng, steer_axis=np.array([-1.0, 0.0, 0.0]),
                           rocker_axis=(np.asarray(DEFAULT_DWB_POINTS["RCK_AX_B"], float)
                                        - np.asarray(DEFAULT_DWB_POINTS["RCK_AX_A"], float)))
    bushings = make_bushings([], DEFAULT_DWB_POINTS)
    far = np.array([9.9e5, 0.0, 0.0])   # 故意无法映射到任何机构节点的车身端 — chassis end deliberately unmappable to any mechanism node
    with __import__("pytest").raises(ValueError, match="does not map to any mechanism node"):
        solve_compliance_full(
            mech, bushings=bushings, case=QSLoad(fy=-3000.0),
            travel=0.0, rack=0.0,
            mk_links=lambda m: [LinkForce(a=far, b=far.copy(), id="PUSH")])
