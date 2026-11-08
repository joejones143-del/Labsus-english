/* =====================================================================
   15-mpc.js — G30 MPC 路径跟踪控制器（2026-09-04） — G30 MPC path-tracking controller (2026-09-04)

   ── 为什么重写（test_lap_15dof 两项哨兵 FAIL 的实证根因）────────── — why the rewrite (evidence-backed root cause of the two sentinel FAILs in test_lap_15dof) ──────
   Stanley 纯几何反馈在"慢速发卡 + 抓地极限"工况处于稳定边界： — Stanley pure-geometric feedback sits on a stability boundary in the "slow hairpin + grip limit" load case:
   弯心 ay 贴死 0.85·μg 包络（1.15g）时，出弯残余横摆 42°/s 在抓地饱和下 — with corner-mid ay pinned at the 0.85·μg envelope (1.15g), the 42°/s residual yaw on exit cannot decay naturally under grip saturation:
   无法自然衰减，车头过转 17° 后反锁救车 → 前轮 α 峰 21°；trail-brake 内侧 — after 17° of over-rotation the counter-lock recovery kicks in → front slip angle peaks at 21°; with trail-brake the inner front wheel unloads
   前轮卸荷（Fz=静载 17%）抱死 κ≈-0.9。5 次修补实验（TCS/速度钳/横摆阻尼 — (Fz = 17% of static load) and locks up at κ≈-0.9. Five patch experiments (TCS / speed clamp / yaw damping
   ×2/ABS 单通道）均失败或代价不可接受，详见 docs/superpowers/specs/ — ×2 / single-channel ABS) all failed or cost too much; see docs/superpowers/specs/
   2026-09-04-mpc-controller-design.md §1。 — 2026-09-04-mpc-controller-design.md §1.

   ── 架构：采样式 CEM-MPC ───────────────────────────────────────── — architecture: sampling-based CEM-MPC ──────
   预测模型 = 平面自行车 + 轮胎线性刚度 + 摩擦圆钳位（饱和可预测——这正是 — prediction model = planar bicycle + linear tire stiffness + friction-circle clamp (saturation is predictable — exactly what
   Stanley 缺的能力）；求解器 = Cross-Entropy-Method 采样式 MPC（规格原定 — Stanley lacks); solver = Cross-Entropy-Method sampling MPC (the spec originally called for
   iLQR，实现层等价替换为 CEM：无导数、无 Riccati、对钳位不连续天然鲁棒， — iLQR; the implementation substitutes CEM as an equivalent: derivative-free, no Riccati, naturally robust to clamp discontinuities,
   K=48 采样 × 2 轮 ≈ 3k 次 dynStep/帧，240Hz 实时无压力）。控制序列热启动 — K=48 samples × 2 rounds ≈ 3k dynSteps/frame, comfortably real-time at 240Hz). The control sequence is warm-started
   （上一帧解平移），含上帧解的候选保证代价单调不升。 — (previous solution shifted); including the previous solution as a candidate guarantees the cost never rises.

   车身系符号约定（模型内部）：+u 前、+v 右、+r 右转；δ 正 = 右转。 — body-frame sign convention (model-internal): +u forward, +v right, +r clockwise (right turn); positive δ = right turn.
   路径曲率 refCurvature 正 = 左转 → 接线时取 κ_m = −refCurvature。 — path curvature refCurvature positive = left turn → the wiring uses κ_m = −refCurvature.
   输出 steer(deg) = −δ_m·R2D（引擎 δ 正 = 左转，与 Stanley 输出同号）。 — output steer(deg) = −δ_m·R2D (engine-positive δ = left turn, same sign as the Stanley output).

   引擎/Python 零改动（双端 parity 安全）；无 DOM 依赖（沙箱可测）。 — zero changes to engine/Python (parity-safe on both sides); no DOM dependency (sandbox-testable).
   F-61：window 导出与定义同文件。 — F-61: window export and definition in the same file.
   ===================================================================== */
"use strict";

/* ── 1. 平面自行车模型 + 轮胎摩擦圆钳位 — 1. planar bicycle model + tire friction-circle clamp ─────────────── */
const MPCModel = {
  params(S) {
    const wb = (S.wb || 2600) / 1000;
    return {
      m: S.mTotal || 1250,
      lf: wb * 0.46, lr: wb * 0.54,
      Iz: 1900,
      mu: 1.35, g: 9.81,
      Cf: 95000, Cr: 110000,          // N/rad（前/后轴合成侧偏刚度，真实量级 15~25kN/rad 按轴) — combined front/rear-axle cornering stiffness; real-world order 15~25 kN/rad per axle)
      axMax: 8.5, brkMax: 12.0,        // m/s²
      N: 40, dt: 0.1,                  // 时域 4s ≈ 200m@50m/s——必须覆盖制动距离 — 4 s horizon ≈ 200 m at 50 m/s — must cover the braking distance
      // （旧 N=30×0.05=1.5s≈75m 短于 T1 制动距离 ~95m，MPC 看不到弯 → 晚刹车冲出） — (old N=30×0.05=1.5s≈75m was shorter than the ~95m T1 braking distance; the MPC could not see the corner → late braking, run-off)
      brkEnv: 11.0,                    // 制动包络减速度（≈0.85·μg） — braking-envelope deceleration (≈0.85·μg)
      aLatLim: 8.5,                    // 参考速度的模型自摩擦极限（≈0.87g）： — the own-model friction limit used for reference speeds (≈0.87g):
      // 路径 v_max 对真车不忠实（diag 实测 s≈3970 高速小曲率弯 u=47.4 滑出）—— — the path v_max is unfaithful to the real car (diag measured s≈3970: high-speed low-curvature corner, u=47.4 slid off) —
      // MPC 应信任自己模型的摩擦极限而非路径乐观包络 — the MPC should trust the friction limit of its own model, not the optimistic path envelope
      vScale: 0.85,                    // 弯速安全系数：v_max 按 0.85·μg 算，但真车含载荷 — corner-speed safety factor: v_max computed at 0.85·μg, but the real car includes load
      // 转移/轮胎载荷敏感只能 ~1.0g——diag 实测每弯 α 20~28° 滑过去的根因 — transfer / tire load sensitivity caps it at ~1.0g — root cause of the diag-measured slip angles of 20~28° per corner
      weights: { q_ey: 8, q_epsi: 1.5, q_v: 0.6, r_delta: 0.02, r_ax: 0.01, r_ddelta: 0.0005, q_soft: 40 }
    };
  },

  clamp(x, lo, hi) { return x < lo ? lo : (x > hi ? hi : x); },

  /* 单步动力学。refK = 该步路径参考曲率（模型内部符号，正 = 右转）。
     x=[u,v,r], w=[δ(rad), ax(m/s²)] → 新 x；out（可选）回传轮胎力用于测试/诊断
     — single-step dynamics. refK = the path reference curvature for this step (model-internal sign, positive = right turn).
     x=[u,v,r], w=[δ(rad), ax(m/s²)] → new x; out (optional) returns tire forces for tests/diagnostics */
  step(x, w, dt, refK, p, out) {
    let [u, v, r] = x;
    const delta = this.clamp(w[0], -0.49, 0.49);
    const ax = this.clamp(w[1], -p.brkMax, p.axMax);
    const uu = Math.max(1.0, u);

    const aF = (v + p.lf * r) / uu - delta;   // ISO：αf = atan((v+a·r)/u) − δ（前轴滑移角，力反向） — ISO slip: front-axle slip angle, force acts opposite
    const aR = (v - p.lr * r) / uu;           // αr = atan((v−b·r)/u)

    // 静态轴载 + 纵向转移一阶（制动前增/驱动后减，hcg/wb≈0.05） — static axle loads + first-order longitudinal transfer (front gains under braking / rear loses under drive, hcg/wb≈0.05)
    const Fz0f = p.m * p.g * p.lr / (p.lf + p.lr);
    const Fz0r = p.m * p.g * p.lf / (p.lf + p.lr);
    const dFz = ax * p.m * 0.05;
    const Fzf = Math.max(p.m * 0.30 * p.g, Fz0f + dFz);
    const Fzr = Math.max(p.m * 0.30 * p.g, Fz0r - dFz);

    // 纵向力按静载比例分轴 + 各自摩擦圆钳位（摩擦椭圆由侧向钳位自然涌现） — longitudinal force split per axle by static-load share, each with its own friction-circle clamp (the friction ellipse emerges naturally from the lateral clamps)
    const Fxf = this.clamp(ax * p.m * 0.46, -p.mu * Fzf, p.mu * Fzf);
    const Fxr = this.clamp(ax * p.m * 0.54, -p.mu * Fzr, p.mu * Fzr);
    const Fyf = this.clamp(-p.Cf * aF, -p.mu * Fzf, p.mu * Fzf);
    const Fyr = this.clamp(-p.Cr * aR, -p.mu * Fzr, p.mu * Fzr);

    const FyfBody = Fyf * Math.cos(delta);
    const u2 = u + (ax + v * r) * dt;   // ISO：u̇ = a_x + r·v
    const v2 = v + ((FyfBody + Fyr) / p.m - u * r) * dt;   // v̇ = Fy/m − u·r（离心项是加速度，不随 m 除） — the centrifugal term is an acceleration and is not divided by m
    const r2 = r + (p.lf * FyfBody - p.lr * Fyr) / p.Iz * dt;
    if (out) { out.Fyf = Fyf; out.Fyr = Fyr; out.Fzf = Fzf; out.Fzr = Fzr; out.Fxf = Fxf; out.Fxr = Fxr; }

    // Frenet 误差传播（参考曲率 κ_ref：ė_ψ = r − u·κ_ref） — Frenet error propagation (reference curvature κ_ref: ė_ψ = r − u·κ_ref)
    const dey = uu * Math.sin(x[4]) + v * Math.cos(x[4]);
    const depsi = r - uu * (refK || 0);

    return [u2, v2, r2, x[3] + dey * dt, x[4] + depsi * dt];
  },

  /* 单步代价（e_y/e_ψ 越界软罚 + 跟踪 + 控制量） — per-step cost (soft penalty for e_y/e_ψ excursion + tracking + control effort) */
  stepCost(x, w, ref, p, W) {
    const q = p.weights;
    const ey = x[3], epsi = x[4];
    const over = Math.max(0, Math.abs(ey) - (ref.bnd || 5.0));
    let J = q.q_ey * ey * ey + q.q_epsi * epsi * epsi +
            q.q_v * (ref.u_ref - x[0]) * (ref.u_ref - x[0]) +
            q.r_delta * w[0] * w[0] + q.r_ax * w[1] * w[1] +
            q.q_soft * over * over;
    if (W && ref.kIdx > 0) {   // 控制增量平滑 — control-increment smoothing
      const dd = w[0] - W[ref.kIdx - 1][0];
      J += q.r_ddelta * dd * dd;
    }
    return J;
  },

  /* CEM 求解：返回 { W, J }。Wprev 热启动（平移），含其自身候选 → 代价单调不升 — CEM solve: returns { W, J }. Wprev is the warm start (shifted); including it as its own candidate → cost never increases */
  solve(x0, Wprev, ref, p, rngState) {
    const N = ref.length, K = 32, ITERS = 1, ELITE = 10;
    // G30：种子化 PRNG（mulberry32）——Math.random 使每次运行行为不同， — G30: seeded PRNG (mulberry32) — Math.random makes every run behave differently,
    // 摆振冲出无法复现/定位；确定性采样让整定可迭代 — shimmery run-offs cannot be reproduced or located; deterministic sampling makes tuning iterable
    let rs = (rngState === undefined ? 123456789 : rngState) >>> 0;
    const rnd = () => {
      rs |= 0; rs = (rs + 0x6D2B79F5) | 0;
      let t = Math.imul(rs ^ (rs >>> 15), 1 | rs);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return (((t ^ (t >>> 14)) >>> 0) / 4294967296) * 2 - 1;
    };
    let mu = Wprev.map(w => w.slice());
    let sig = Wprev.map(() => [0.005, 0.8]);   // G30：δ 探索噪声 0.3°——直道 47m/s 处 — G30: steering exploration noise of 0.3° — at 47 m/s on a straight
    // CEM 每帧 δ 抖动（旧 σ=0.02≈1.1°@240Hz）激励 1.5-2Hz 横摆/侧倾耦合振荡至自旋； — CEM per-frame δ jitter (old σ=0.02≈1.1°@240Hz) excited a 1.5–2Hz yaw/roll coupled oscillation up to spin-out;
    // 横向律由解析种子提供，CEM 只精修纵向 — the lateral law is provided by the analytic seed; CEM only refines the longitudinal channel
    let bestW = null, bestJ = Infinity;

    for (let it = 0; it < ITERS; it++) {
      const cand = [];
      // 候选 0：热启动自身（保证不劣化） — candidate 0: the warm start itself (guarantees no degradation)
      cand.push([mu.map(w => w.slice()), this.rolloutCost(x0, mu, ref, p)]);
      for (let k = 1; k < K; k++) {
        const Wk = [];
        for (let t = 0; t < N; t++) {
          const sc = (it === 0) ? 1.0 : 0.7;
          Wk.push([
            this.clamp(mu[t][0] + rnd() * sc * sig[t][0], -0.49, 0.49),
            this.clamp(mu[t][1] + rnd() * sc * sig[t][1], -p.brkMax, p.axMax)
          ]);
        }
        cand.push([Wk, this.rolloutCost(x0, Wk, ref, p)]);
      }
      cand.sort((a, b) => a[1] - b[1]);
      if (cand[0][1] < bestJ) { bestJ = cand[0][1]; bestW = cand[0][0].map(w => w.slice()); }
      // 精英重拟合 — elite refit
      for (let t = 0; t < N; t++) {
        let m0 = 0, m1 = 0;
        for (let e = 0; e < ELITE; e++) { m0 += cand[e][0][t][0]; m1 += cand[e][0][t][1]; }
        mu[t][0] = m0 / ELITE; mu[t][1] = m1 / ELITE;
        let s0 = 0, s1 = 0;
        for (let e = 0; e < ELITE; e++) {
          s0 += (cand[e][0][t][0] - mu[t][0]) ** 2;
          s1 += (cand[e][0][t][1] - mu[t][1]) ** 2;
        }
        sig[t][0] = Math.max(0.01, Math.sqrt(s0 / ELITE));
        sig[t][1] = Math.max(0.10, Math.sqrt(s1 / ELITE));
      }
    }
    return { W: bestW, J: bestJ, rng: rs >>> 0 };
  },

  rolloutCost(x0, W, ref, p) {
    let x = x0.slice(), J = 0;
    for (let t = 0; t < ref.length; t++) {
      x = this.step(x, W[t], p.dt, ref[t].kappa, p);
      J += this.stepCost(x, W[t], ref[t], p, W);
    }
    // 终端代价（×8）：出口误差与速度偏差 — terminal cost (×8): exit errors and speed deviation
    const xf = x;
    const refE = ref[ref.length - 1];
    J += 8 * (p.weights.q_ey * xf[3] * xf[3] + p.weights.q_epsi * xf[4] * xf[4] +
              p.weights.q_v * (refE.u_ref - xf[0]) * (refE.u_ref - xf[0]));
    return J;
  }
};

/* ── 2. MPC 控制器（与 UniversalAutoPilot 同 API） — 2. MPC controller (same API as UniversalAutoPilot) ───────────── */
class UniversalAutoPilotMPC {
  constructor(S_config) {
    const wb = (S_config.wb || 2600) / 1000;
    this.wb = wb;
    this.a = wb * 0.46;
    this.pid_speed = { p: 0.95, i: 0.08, d: 0.04, integral: 0, last_err: 0 };
    this.last_steer = 0;
    this.last_throttle = 0;
    this.last_brake = 0;
    this.active = false;
    this.path = null;
    this.kLat = 1.8;
    this.Scfg = S_config;
    // G21 学习（接口与 Stanley 完全一致） — G21 learning (interface identical to Stanley)
    this.cornerMargins = {};
    this.cornerLock = {};
    this.cornerDirty = {};
    // G30：滑移护栏（发生 |v|>0.45 滑移的弯冻结 margin 推进） — G30: slide guardrail (corners where |v|>0.45 slip occurs freeze margin advancement)
    this.cornerSlide = {};
    // G30 静轮载（ABS 卸荷通道） — G30 static wheel loads (ABS unloading channel)
    const w0 = (S_config.mTotal || 1250) * 9.81;
    this.Fz0F = w0 * 0.54 / 2;
    this.Fz0R = w0 * 0.46 / 2;
    // MPC 状态 — MPC state
    this._W = null;          // 热启动控制序列 — warm-started control sequence
    this._s = 0;             // 当前弧长（由 lookahead 回填） — current arc length (backfilled by the lookahead)
    this._prm = null;
  }

  initLapLearning(nCorners) {
    this.cornerMargins = {};
    this.cornerLock = {};
    this.cornerDirty = {};
    this.cornerSlide = {};
    for (let i = 0; i < (nCorners || 0); i++) this.cornerMargins[i] = 0.85;
  }

  reportRunoff(cid) { if (cid >= 0) this.cornerDirty[cid] = true; }

  /* G21 圈末结算 + G30 滑移护栏：发生侧滑的弯冻结推进，连续侧滑回退锁定 — per-lap bookkeeping (G21) + G30 slide guardrail: corners that slid freeze advancement; repeated slides roll back the lock */
  endLap() {
    for (const k in this.cornerMargins) {
      const i = +k;
      if (this.cornerDirty[i]) {
        this.cornerMargins[i] = Math.max(0.70, this.cornerMargins[i] - 0.12);
        this.cornerLock[i] = true;
      } else if (this.cornerSlide[i]) {
        // 冻结：不推进也不回退；连续两圈侧滑才回退（滑移计数 ≥2） — freeze: neither advance nor roll back; roll back only after two consecutive sliding laps (slide count ≥2)
        if (this.cornerSlide[i] >= 2) {
          this.cornerMargins[i] = Math.max(0.70, this.cornerMargins[i] - 0.12);
          this.cornerLock[i] = true;
          this.cornerSlide[i] = 0;
        }
      } else if (!this.cornerLock[i]) {
        this.cornerMargins[i] = Math.min(1.0, this.cornerMargins[i] + 0.07);
      }
    }
    this.cornerDirty = {};
  }

  setPath(newPath) {
    this.path = newPath;
    this.pid_speed.integral = 0;
    this.pid_speed.last_err = 0;
    this.last_steer = 0;
    this.last_throttle = 0;
    this.last_brake = 0;
    this._W = null;
    this._s = 0;
    this._prm = null;
  }

  drive(state, dt) {
    if (!this.path || !this.active) return { steer: 0, throttle: 0, brake: 0, target: null };

    // G30 性能：每帧求解（旧隔帧保持 8ms 在高速下与植物滞后叠加引发 2-3Hz 摆振） — G30 performance: solve every frame (the old every-other-frame 8 ms hold, superimposed on plant lag at high speed, caused 2–3Hz shimmy)
    this._f = (this._f || 0) + 1;
    const reSolve = true;
    if (!reSolve) {
      const c = this._lastCtrl;
      const tgt = this.path.getLookahead(
        state.X - this.a * Math.sin(state.psi), state.Y + this.a * Math.cos(state.psi), state.u);
      return { steer: c.steer, throttle: c.throttle, brake: c.brake, drs: c.drs, target: tgt };
    }

    const u = Math.max(0.1, state.u || 0);
    const fx = state.X - this.a * Math.sin(state.psi);
    const fy = state.Y + this.a * Math.cos(state.psi);

    const target = this.path.getLookahead(fx, fy, u);
    this._s = (target && target.sRef !== undefined) ? target.sRef
            : (target && target.s !== undefined ? target.s : this._s);

    if (!this._prm) this._prm = MPCModel.params(this.Scfg || {});
    const p = this._prm;

    // ── 参考采样（一致时域）：按热启动预测速度积分弧长逐点采样。 — reference sampling (consistent horizon): integrate arc length with the warm-start-predicted speed and sample point by point.
    //   旧实现 dsStep = max(1.5, u·dt) 的下限在低速时把参考线拉到车实际行程的 — the old dsStep = max(1.5, u·dt) floor pulled the reference line several times beyond the actual travel of the car at low speed
    //   数倍之外（u=5 时模型 30 步只走 7.5m，参考却采到 45m 外）——远处弯角的 — (u=5: the model covers 7.5 m in 30 steps while the reference sampled 45 m ahead) — the low v_max of far-away corners
    //   低 v_max 会让 MPC 主动爬行失速（calib 实测 12s 内 u 卡在 ~6m/s 的根因）。 — made the MPC crawl and stall deliberately (root cause: calib measured u stuck at ~6 m/s for 12 s).
    const N = p.N;
    const ref = [];
    {
      let sAcc = this._s;
      let uPred = u;
      for (let k = 0; k < N; k++) {
        sAcc += Math.max(0.5, uPred) * p.dt;
        const rp = this.path.samplePath(sAcc, 1, 1)[0];
        const cid = rp.cornerId;
        const margin = (cid >= 0 && this.cornerMargins[cid] !== undefined) ? this.cornerMargins[cid] : 1.0;
        // G30 滑移护栏：发生侧滑的弯不再被 margin 推到更高目标速（endLap 冻结/回退） — G30 slide guardrail: corners that slid are no longer pushed faster by the margin (endLap freezes/rolls back)
        const slid = this.cornerSlide[cid] !== undefined && this.cornerSlide[cid] > 0;
        const uCapK = Math.sqrt(p.aLatLim / Math.max(1e-4, Math.abs(rp.curvature || 0)));
        ref.push({
          u_ref: Math.min(rp.v_max * (slid ? Math.min(margin, 0.88) : margin) * p.vScale, uCapK),
          kappa: (rp.curvature || 0),        // 模型与路径同约定：正 = 左转（ISO r 正） — same convention as the path: positive = left turn (ISO positive yaw rate)
          bnd: (this.path.hw_m || 4.9) + (this.path.kerb_m || 1.35) - 0.5,
          s: sAcc,
          kIdx: k
        });
        uPred = MPCModel.clamp(uPred + ((this._W && this._W[k]) ? this._W[k][1] : 0) * p.dt, 1, 60);
      }
      // 制动距离反向包络：u_ref[k] = min(u_ref[k], sqrt(u_ref[k+1]² + 2·a_brk·Δs)) — braking-distance reverse envelope: u_ref[k] = min(u_ref[k], sqrt(u_ref[k+1]² + 2·a_brk·Δs))
      // ——让 MPC 在制动点就开始减速，而不是等弯进入时域 — so the MPC starts slowing at the braking point instead of waiting for the corner to enter the horizon
      for (let k = N - 2; k >= 0; k--) {
        const ds = Math.max(0, ref[k + 1].s - ref[k].s);
        const vCap = Math.sqrt(Math.max(0, ref[k + 1].u_ref * ref[k + 1].u_ref + 2 * p.brkEnv * ds));
        if (vCap < ref[k].u_ref) ref[k].u_ref = vCap;
      }
    }

    // ── CEM 求解 ──
    // 误差态注入：符号由 _errSign 校准（模型 ISO 系 vs 引擎约定的跨系映射， — error-state injection: signs calibrated via _errSign (cross-frame mapping between the ISO frame of the model and the engine convention,
    //   手推易错，采用经验校准：scratch/calib_mpc_signs.cjs 网格搜索 4 种组合） — deriving by hand is error-prone, so use empirical calibration: scratch/calib_mpc_signs.cjs grid-searched 4 combinations)
    const es = this._errSign || { ey: -1, epsi: 1 };
    if (!this._W || this._W.length !== N) this._W = ref.map(() => [0, 0]);
    const e_y0 = (target && typeof target.lineError === "number" && isFinite(target.lineError)) ? es.ey * target.lineError : 0;
    let e_psi0 = (target ? es.epsi * (state.psi - target.targetHeading) : 0);
    while (e_psi0 > Math.PI) e_psi0 -= 2 * Math.PI;
    while (e_psi0 < -Math.PI) e_psi0 += 2 * Math.PI;

    // ── 解析热启动（CEM 只做精修，不做发现）── — analytic warm start (CEM only refines, it does not discover) ──
    // 纯采样搜索在 5 维问题上发现能力不足（探针实测：e_y→δ 几乎无响应、 — pure sampling search lacks discovery power on this 5-D problem (probe measured: e_y→δ barely responds,
    // 纵向因横向失败主动降速失速）。横向用 Stanley 型前馈+反馈（符号经单步 — and the longitudinal channel deliberately slows down and stalls once the lateral fails). Lateral uses a Stanley-type feedforward+feedback (signs verified with a single-step probe:
    // 探针验证：e_y>0/e_ψ>0 → δ<0；κ>0 左弯 → δ>0），纵向用速度误差比例。 — e_y>0/e_ψ>0 → δ<0; κ>0 left turn → δ>0); longitudinal uses a proportional speed error.
    const seed = [];
    {
      let ey = e_y0, epsi = e_psi0, uu2 = u;
      // G30 整定：增益随速度调度（高速降权威，防延迟诱导 PIO——起步直道 u≈28 摆振） — G30 tuning: speed-scheduled gains (reduce authority at high speed to prevent delay-induced PIO — shimmy on the opening straight at u≈28)
      const gSched = Math.min(1.0, 20 / Math.max(8, u));
      const K_E = 0.35 * gSched, K_PSI = 0.9 * gSched;
      // 横摆率阻尼：δ 补偿 (r − u·κ_ref)——参考曲率已正确后此项安全 — yaw-rate damping: δ compensation of (r − u·κ_ref) — safe now that the reference curvature is correct
      // （早期尝试发散是因为当时用了前视点曲率，前置条件已消除） — (the earlier divergence came from using look-ahead-point curvature back then; that pre-condition is gone)
      const K_R = 0.25 * gSched;
      const rErr0 = (state.r || 0) - u * (ref[0].kappa || 0);
      for (let k = 0; k < N; k++) {
        const dff = Math.atan(this.wb * (ref[k].kappa || 0));
        const dLat = -K_E * ey - K_PSI * epsi - K_R * rErr0;
        const dSeed = MPCModel.clamp(dff + dLat, -0.15, 0.15);
        const aSeed = MPCModel.clamp((ref[k].u_ref - uu2) * 1.5, -p.brkMax, p.axMax);
        seed.push([dSeed, aSeed]);
        // 沿时域传播误差（运动学自行车近似） — propagate the errors along the horizon (kinematic bicycle approximation)
        ey += (uu2 * Math.sin(epsi) + (state.v || 0)) * p.dt;
        epsi += (uu2 * dSeed / this.wb - uu2 * (ref[k].kappa || 0)) * p.dt;
        uu2 = MPCModel.clamp(uu2 + aSeed * p.dt, 1, 60);
      }
    }
    const sol = MPCModel.solve([u, state.v || 0, state.r || 0, e_y0, e_psi0], seed, ref, p, this._rng);
    this._rng = sol.rng;
    this._W = sol.W.map(w => w.slice());
    // 热启动平移（下一帧从 t=1 起步，尾部补常值） — warm-start shift (next frame starts at t=1; the tail is padded with a constant)
    this._W.shift();
    this._W.push(this._W[this._W.length - 1].slice());

    const w0 = sol.W[0];
    const deltaM = w0[0];               // 模型符号（正 = 右转） — model sign (positive = right turn)
    const axM = w0[1];

    // ── 踏板映射 ── — pedal mapping ──
    let target_throttle = 0, target_brake = 0;
    if (axM > 0.15) target_throttle = Math.min(1.0, axM / 8.0);
    else if (axM < -0.30) target_brake = Math.min(1.0, -axM / p.brkMax);

    // Cornering TCS（保留 Stanley 版语义：油门时侧滑削油） — cornering TCS (keeps the Stanley-version semantics: cut throttle on lateral slip while on throttle)
    const v_lat = Math.abs(state.v || 0);
    if (target_throttle > 0 && v_lat > 0.40) {
      target_throttle = Math.max(0, target_throttle - (v_lat - 0.40) * 2.2);
    }

    // ── 转向：模型 δ → 引擎 steer（反号）+ 低通 + 速率限制（防 CEM 抖动与 PIO）── — steering: model δ → engine steer (sign-flipped) + low-pass + rate limit (guards against CEM jitter and PIO) ──
    let steerDeg = -deltaM * (180 / Math.PI);
    steerDeg = Math.max(-28, Math.min(28, steerDeg));
    const steer_alpha = Math.min(1.0, dt / 0.10);
    steerDeg = this.last_steer + (steerDeg - this.last_steer) * steer_alpha;
    const max_steer_rate = 100.0;   // deg/s（Stanley 用 140，MPC 权威更大需更紧） — deg/s (Stanley uses 140; with the larger MPC authority, tighter is needed)
    const d_steer = Math.max(-max_steer_rate * dt, Math.min(max_steer_rate * dt, steerDeg - this.last_steer));
    const ctrl_steer = this.last_steer + d_steer;
    this.last_steer = ctrl_steer;

    // 1st-order smoothing（与 Stanley 同款，保留踏板平滑） — 1st-order smoothing (same as Stanley, keeps pedal smoothing)
    const filter_alpha = Math.min(1.0, dt / 0.045);
    let ctrl_throttle = this.last_throttle + (target_throttle - this.last_throttle) * filter_alpha;
    let ctrl_brake = this.last_brake + (target_brake - this.last_brake) * filter_alpha;
    if (ctrl_throttle < 0.015) ctrl_throttle = 0;
    if (ctrl_brake < 0.015) ctrl_brake = 0;

    // G30：油门/制动变化率限制——diag 实测直道 u=47m/s 处油门 1→0 在 ~0.15s 内 — G30: throttle/brake rate limits — diag measured throttle 1→0 completing in ~0.15 s on a straight at u=47 m/s
    // 完成（低通 τ=45ms），瞬态载荷转移激发后轴 lift-off 甩尾（v→23m/s 自旋）。 — (low-pass τ=45ms); the transient load transfer excited a rear lift-off spin-out (v→23 m/s spin).
    // 油门松取限 2.5/s、制动施加限 4.0/s，让载荷转移平缓；制动释放不限（安全侧）。 — throttle release limited to 2.5/s, brake application to 4.0/s, to soften load transfer; brake release unlimited (safe side).
    ctrl_throttle = MPCModel.clamp(ctrl_throttle,
      this.last_throttle - 2.5 * dt, this.last_throttle + 6.0 * dt);
    ctrl_brake = MPCModel.clamp(ctrl_brake,
      this.last_brake - 8.0 * dt, this.last_brake + 4.0 * dt);

    // ── G30 ABS 输出级（保留：κ/轮载骤降双通道，绕过低通）── — G30 ABS output stage (kept: κ / wheel-load-drop dual channels, bypassing the low-pass) ──
    const pedalBrake = ctrl_brake;
    const telA = (this.eng && this.eng.telemetry && this.eng.state === state) ? this.eng.telemetry : null;
    if (telA && ctrl_brake > 0) {
      let kMax = 0, unload = 0;
      for (const w of ['FL', 'FR', 'RL', 'RR']) {
        kMax = Math.max(kMax, Math.abs(telA.kappa[w] || 0));
        const fz0w = (w[0] === 'F' ? this.Fz0F : this.Fz0R) || 1;
        unload = Math.max(unload, 1 - Math.min(1, (telA.Fz[w] || 0) / (0.30 * fz0w)));
      }
      const risk = Math.max(kMax > 0.12 ? (kMax - 0.12) * 2.0 : 0, unload);
      if (risk > 0) ctrl_brake *= Math.max(0.10, 1 - risk);
    }
    this.last_brake = pedalBrake;

    // ── G30 滑移护栏记录：|v|>0.45 的弯计入 cornerSlide（圈末冻结 margin）── — G30 slide-guardrail record: corners with |v|>0.45 count into cornerSlide (margin frozen at lap end) ──
    const telS = (this.eng && this.eng.telemetry && this.eng.state === state) ? this.eng.telemetry : null;
    const cidNow = (target && typeof target.cornerId === "number") ? target.cornerId : -1;
    if (telS && cidNow >= 0 && v_lat > 0.45) {
      this.cornerSlide[cidNow] = (this.cornerSlide[cidNow] || 0) + 1;
    }

    // DRS（与 Stanley 同口径） — DRS (same criteria as Stanley)
    const qsD = (this.Scfg && this.Scfg.qs) || {};
    const drsVMin = (typeof qsD.drsVms === 'number' && isFinite(qsD.drsVms)) ? qsD.drsVms : 40.0;
    const ctrl_drs = !!(target && target.isDRS && u > drsVMin);

    // G30 修复：踏板平滑状态必须回写（此前丢失导致每帧从 0 起步，thr 恒为单步平滑值 0.09） — G30 fix: the pedal-smoothing state must be written back (losing it previously made every frame start from 0, leaving thr stuck at the single-step smoothed value 0.09)
    this.last_throttle = ctrl_throttle;

    const out = { steer: ctrl_steer, throttle: ctrl_throttle, brake: ctrl_brake, drs: ctrl_drs, target: target };
    this._lastCtrl = { steer: ctrl_steer, throttle: ctrl_throttle, brake: ctrl_brake, drs: ctrl_drs };
    return out;
  }
}

if (typeof window !== "undefined") {
  window.MPCModel = MPCModel;
  window.UniversalAutoPilotMPC = UniversalAutoPilotMPC;
}
