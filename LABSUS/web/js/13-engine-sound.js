/* =====================================================================
   13-engine-sound.js —— 引擎声浪系统（P2-Sound，2026-09-02） — engine sound system (P2-Sound, 2026-09-02)

   程序化合成（零素材）：Web Audio 谐波堆 + 排气噪声实时合成，全参数 — procedural synthesis (zero assets): Web Audio harmonic stack + real-time exhaust noise, all parameters
   与工况联动。两层架构： — coupled to the load case. Two-layer architecture:

   · EngineAudioModel —— 纯逻辑（headless 可测）： — pure logic (testable headless):
       挡位状态机（6 挡齿轮比 × 终传比，RPM 由驱动轮 omega 反推）， — gear state machine (6 gear ratios × final drive, RPM back-computed from driven-wheel omega),
       升/降挡迟滞 + 换挡断油 cut，声学参数映射（点火频率/谐波比/亮度/ — up/downshift hysteresis + shift fuel-cut, acoustic parameter mapping (firing frequency / harmonic ratios / brightness /
       增益/爆燃回火事件）。无 DOM / 无 AudioContext 依赖。 — gain / backfire events). No DOM / no AudioContext dependency.
   · EngineAudioSink —— Web Audio 封装： — Web Audio wrapper:
       惰性创建（浏览器 autoplay 政策：须用户手势解锁），setTargetAtTime — lazily created (browser autoplay policy: needs a user gesture to unlock), setTargetAtTime
       平滑防 zipper noise；无 AudioContext（vm 沙箱/旧浏览器）时 inert。 — smoothing prevents zipper noise; inert without AudioContext (vm sandbox / old browsers).

   声线随车型预设（VEHICLE_PRESETS.vehicleType）变化： — the sound profile changes with the vehicle preset (VEHICLE_PRESETS.vehicleType):
       formula = FSAE 600cc I4 高转嘶吼（idle 3000 / redline 11000） — high-revving shriek (idle 3000 / redline 11000)
       gt3     = V8 4.0 NA 低吼    （idle 950  / redline 9000） — deep V8 growl (idle 950 / redline 9000)
       gt3_sport = I4 turbo        （idle 850  / redline 7200） — I4 turbo whistle (idle 850 / redline 7200)
       baja    = 单缸 450 突突     （idle 1500 / redline 8200） — single-cylinder 450 thump (idle 1500 / redline 8200)

   工况联动：油门→音量/亮度/谐波比；换挡→RPM 断落 + 断油闷响；重刹 — load-case coupling: throttle→volume/brightness/harmonic ratio; shift→RPM drop + fuel-cut muffle; heavy-brake
   降挡→排气回火 pop；驱动轮打滑（|κ|>0.12）→轮胎啸叫层；路肩→音量 — downshift→exhaust pop; driven-wheel slip (|κ|>0.12)→tire squeal layer; kerb→volume
   抖动调制。三舞台（SLOPE/SKIDPAD/CIRCUIT）loop 每帧喂 — flutter modulation. The three stage (SLOPE/SKIDPAD/CIRCUIT) loops feed
   EngineSound.update(dt, st, tel, ctrl)；watchdog 250ms 无喂自动淡出 — EngineSound.update(dt, st, tel, ctrl) every frame; watchdog auto-fades after 250 ms without feeding
   （舞台切走/暂停不残留空转轰鸣）。 — (no idling roar left behind when switching/pausing a stage).

   F-61 约束：window 导出与定义同文件；顶层无 AudioContext 也不抛错。 — F-61 constraint: window exports and definitions in the same file; no throw at top level without AudioContext either.
   ===================================================================== */
"use strict";

/* ── 声线档案（per 车型预设）────────────────────────────────────────── — sound profile catalog (per vehicle preset) ──────
   cyl：缸数（点火频率 = rpm/60 × cyl/2，四冲程）；baseMul：排气主听感 — cyl: cylinder count (firing frequency = rpm/60 × cyl/2, four-stroke); baseMul: main exhaust character multiplier
   倍率（V8 低沉 0.5、单缸突突 2.0）；ratios：各挡齿轮比；final：终传比； — (deep V8 0.5, single-cylinder thump 2.0); ratios: per-gear gear ratios; final: final drive ratio;
   shiftUp/shiftDown：换挡迟滞带（占 redline 比例）。 — shiftUp/shiftDown: shift hysteresis bands (fractions of redline). */
const ENGINE_SOUND_PROFILES = {
  formula:   { label: "FSAE I4",   cyl: 4, idle: 3000, redline: 11000, baseMul: 1.0,
               ratios: [2.80, 2.00, 1.60, 1.30], final: 3.90, shiftUp: 0.95, shiftDown: 0.55 },
  gt3:       { label: "V8 NA",     cyl: 8, idle: 950,  redline: 9000,  baseMul: 0.5,
               ratios: [3.00, 2.20, 1.75, 1.40, 1.15, 0.98], final: 3.70, shiftUp: 0.94, shiftDown: 0.50 },
  gt3_sport: { label: "I4 Turbo",  cyl: 4, idle: 850,  redline: 7200,  baseMul: 1.0,
               ratios: [3.00, 2.20, 1.75, 1.40, 1.15, 0.98], final: 3.70, shiftUp: 0.93, shiftDown: 0.50 },
  baja:      { label: "Single 450", cyl: 1, idle: 1500, redline: 8200,  baseMul: 2.0,
               ratios: [3.00, 2.10, 1.50], final: 4.30, shiftUp: 0.94, shiftDown: 0.55 }
};
window.ENGINE_SOUND_PROFILES = ENGINE_SOUND_PROFILES;

const ENGINE_SOUND_MAX_GEAR_SHIFT_COOLDOWN = 0.30;  // 换挡后防抖冷却 s — post-shift debounce cooldown, seconds
const ENGINE_SOUND_CUT_T = 0.13;                    // 换挡断油时长 s — fuel-cut duration during shift, seconds
const ENGINE_SOUND_RPM_INERTIA = 12.0;              // 引擎转速一阶惯性 (1/s) — first-order engine-speed inertia (1/s)

class EngineAudioModel {
  constructor(profileKey) {
    this.setProfile(profileKey);
    this.gear = 1;
    this.rpm = this.p ? this.p.idle : 900;
    this.cutT = 0;
    this.shiftCooldown = 0;
    this.t = 0;
  }

  setProfile(profileKey) {
    this.profileKey = ENGINE_SOUND_PROFILES[profileKey] ? profileKey : "gt3";
    this.p = ENGINE_SOUND_PROFILES[this.profileKey];
  }

  /* sig: { wheelOmega: 驱动轮角速度 rad/s, throttle, brake, — sig: { wheelOmega: driven-wheel angular velocity rad/s, throttle, brake,
            kappaMax: |κ| 最大值, isKerb: 任一轮上路肩 } → 声学参数帧 — kappaMax: max |κ|, isKerb: any wheel on kerb } → acoustic parameter frame */
  update(dt, sig) {
    const p = this.p;
    this.t += dt;
    this.cutT = Math.max(0, this.cutT - dt);
    this.shiftCooldown = Math.max(0, this.shiftCooldown - dt);

    const thr = Math.max(0, Math.min(1, sig.throttle || 0));
    const brk = Math.max(0, Math.min(1, sig.brake || 0));
    const wheelOmega = Math.max(0, sig.wheelOmega || 0);

    /* M-1(G31-P6-fix)：rpm 源判据改架构感知——POWERTRAIN.hasLiveRpmSource() — M-1(G31-P6-fix): rpm source gate is now architecture-aware — POWERTRAIN.hasLiveRpmSource()
       （spec && state && (architecture==="ev" || hasRealDrivetrain())）为真时才信任动力链 rpm 源， — (spec && state && (architecture==="ev" || hasRealDrivetrain())) must be true before the drivetrain rpm source is trusted,
       与 11-stages.js HUD（renderCircuitTelemetry）共用同一判据，避免两处各写一份、语义漂移。 — sharing the same gate as the 11-stages.js HUD (renderCircuitTelemetry) to avoid two divergent copies drifting apart.
       G31-P10-3：EV rpm 源改用 POWERTRAIN.step 回传的 shiftRpm（驱动电机轴 rpm， — G31-P10-3: the EV rpm source now uses shiftRpm returned by POWERTRAIN.step (drive-motor shaft rpm,
         = 轮速×减速器 ratio×finalDrive，fwd/rwd 架构感知已在 16-powertrain 内算好）—— — = wheel speed × reducer ratio × finalDrive; fwd/rwd awareness is already computed inside 16-powertrain) —
         旧式 sig.wheelOmega×30/π 换算漏乘减速器齿比（把电机轴转速当成轮速）。 — the old sig.wheelOmega×30/π conversion omitted the reducer tooth ratio (treating motor-shaft speed as wheel speed).
         shiftRpm 取自 window.physicsEngine._ptOut（接线层 11-stages.js 每子步存， — shiftRpm comes from window.physicsEngine._ptOut (stored every substep by the wiring layer 11-stages.js,
         最小侵入：不改三舞台 loop 的 update 签名）；缺失/无效（早期帧、其他舞台未跑 — minimal intrusion: the three stage loops' update signatures unchanged); missing/invalid (early frames, other stages not yet running
         step）→ guard 回退旧轮速换算。 — step) → the guard falls back to the old wheel-speed conversion.
       非 EV + 真实传动比：用 state.iceOmega×30/π（曲轴，运动学耦合含齿比）； — non-EV + real drivetrain: use state.iceOmega×30/π (crankshaft, kinematic coupling includes gear ratio);
       legacy 恒等箱 + 非 EV：hasLiveRpmSource()=false → ptRpm=null → 回退旧 — legacy identity gearbox + non-EV: hasLiveRpmSource()=false → ptRpm=null → fall back to the old
         wheel-omega×齿轮比估算（对拍锚，避免塌到怠速）。 — wheel-omega x gear-ratio estimate (parity anchor, avoids collapsing to idle). */
    const ptLive = (typeof POWERTRAIN !== "undefined" && POWERTRAIN.hasLiveRpmSource && POWERTRAIN.hasLiveRpmSource());
    const ptOut = (ptLive && sig.ptOut && Number.isFinite(sig.ptOut.shiftRpm) && sig.ptOut.shiftRpm > 0)
      ? sig.ptOut : null;
    const ptRpm = ptLive
      ? ((POWERTRAIN.spec.architecture === "ev")
          ? ((ptOut !== null) ? ptOut.shiftRpm
                              : wheelOmega * 30.0 / Math.PI)   // EV 无 ptOut → 旧轮速换算 — EV without ptOut → old wheel-speed conversion
          : POWERTRAIN.state.iceOmega * 30.0 / Math.PI)
      : null;

    /* 1. 挡位：G31-P10-3 统一——POWERTRAIN 接管（hasLiveRpmSource）时挡位显示 — 1. gear: unified by G31-P10-3 — when POWERTRAIN takes over (hasLiveRpmSource), the gear display
          直接跟随 POWERTRAIN.state.gearIdx（0-based→1-based），挡位状态机【不推进】 — directly follows POWERTRAIN.state.gearIdx (0-based→1-based) and the gear state machine does NOT advance
          （换挡唯一来源 = POWERTRAIN gearbox 状态机，声浪不再自产 shift/cut/pop）。 — (the only shift source is the POWERTRAIN gearbox state machine; the sound no longer self-produces shift/cut/pop).
          仅回退路径（ptRpm===null：沙箱无 POWERTRAIN / legacy 恒等箱）沿用旧 — only the fallback path (ptRpm===null: sandbox without POWERTRAIN / legacy identity gearbox) keeps the old
          状态机：RPM 由驱动轮 omega × 齿轮比反推（打滑时 omega 飙升 → 转速跟着 — state machine: RPM back-computed from driven-wheel omega × gear ratio (omega spikes under spin → rpm follows
          飙升，与真实驱动轮滑转一致），逐位保留（对拍锚）。 — matching real driven-wheel spin-up, preserved bit-for-bit (parity anchor). */
    let events = null;
    if (ptRpm !== null) {
      const stG = POWERTRAIN.state;
      if (stG && Number.isFinite(stG.gearIdx)) {
        this.gear = Math.max(1, stG.gearIdx + 1);   // 仅作显示；可超 profile.ratios.length — display only; may exceed profile.ratios.length
      }
    } else {
      /* 防御：POWERTRAIN 模式留下的越界挡位（箱挡数 > profile 挡数）先夹回， — defense: out-of-range gear left by POWERTRAIN mode (box gears > profile gears) is clamped back first,
         保 ratios 查表不 NaN（正常回退路径中恒无操作，不改变既有逐位行为） — keeping the ratios lookup NaN-free (always a no-op on the normal fallback path; existing bit-for-bit behavior unchanged) */
      this.gear = Math.max(1, Math.min(p.ratios.length, this.gear));
      const ratio = p.ratios[this.gear - 1] * p.final;
      const rpmRaw = wheelOmega * ratio * 60.0 / (2.0 * Math.PI);
      const rpmRawEff = (!isFinite(rpmRaw) || rpmRaw < p.idle * 0.6) ? p.idle : rpmRaw;
      if (this.shiftCooldown <= 0) {
        const upRpm = p.redline * p.shiftUp;
        const downRpm = p.redline * p.shiftDown;
        if (this.gear < p.ratios.length && rpmRawEff > upRpm && thr > 0.2) {
          this.gear++; this.cutT = ENGINE_SOUND_CUT_T;
          this.shiftCooldown = ENGINE_SOUND_MAX_GEAR_SHIFT_COOLDOWN;
          (events = events || []).push("shift");
        } else if (this.gear > 1 && (rpmRawEff < p.idle * 1.6 ||
                   (brk > 0.1 && rpmRawEff < downRpm))) {
          this.gear--; this.cutT = ENGINE_SOUND_CUT_T;
          this.shiftCooldown = ENGINE_SOUND_MAX_GEAR_SHIFT_COOLDOWN;
          (events = events || []).push("shift");
          if (brk > 0.1) (events = events || []).push("pop");   // 重刹降挡回火 — downshift under hard braking → exhaust backfire
        }
      }
    }

    /* 2. 引擎转速一阶惯性（换挡齿轮比变化 → 目标骤变 → 惯性滑落/拉起） — 2. first-order engine-speed inertia (gear-ratio change → target jumps → inertia-driven fall/rise)
       POWERTRAIN 接管时 rpmTarget = ptRpm（曲轴/电机轴真实转速，PT10-3）。 — under POWERTRAIN takeover rpmTarget = ptRpm (true crankshaft/motor-shaft speed, PT10-3).
       ratioNew 在接管模式下可能越界 NaN，但三元短路下不被消费；回退路径照旧。 — in takeover mode ratioNew can go out of range/NaN, but the ternary short-circuit never consumes it; the fallback path is unchanged. */
    const ratioNew = p.ratios[this.gear - 1] * p.final;
    let rpmTarget = (ptRpm !== null) ? ptRpm : (wheelOmega * ratioNew * 60.0 / (2.0 * Math.PI));
    if (!isFinite(rpmTarget) || rpmTarget < p.idle) rpmTarget = p.idle;
    rpmTarget = Math.min(p.redline * 1.04, rpmTarget);
    this.rpm += (rpmTarget - this.rpm) * Math.min(1.0, dt * ENGINE_SOUND_RPM_INERTIA);

    /* 3. 声学映射 */
    const cutK = this.cutT > 0 ? 0.25 : 1.0;                 // 断油闷响 — fuel-cut muffle
    const thrEff = thr * (this.cutT > 0 ? 0.3 : 1.0);
    const loadK = Math.max(thrEff, brk * 0.25);              // 滑行仍有基础声 — coasting still keeps a base sound
    const fFiring = (this.rpm / 60.0) * (p.cyl / 2.0) * p.baseMul;
    const rpmK = this.rpm / p.redline;
    let gain = 0.10 + 0.30 * loadK + 0.10 * rpmK;
    if (sig.isKerb) gain *= 1.0 + 0.15 * Math.sin(this.t * 55.0);  // 路肩抖动 — kerb flutter
    const kappa = Math.abs(sig.kappaMax || 0);
    const slip = Math.max(0, Math.min(1.0, (kappa - 0.12) * 1.2)); // 轮胎啸叫 — tire squeal

    return {
      f1: fFiring * 0.5, f2: fFiring, f3: fFiring * 2.0,
      h1: 1.0, h2: 0.5 + 0.4 * thrEff, h3: 0.25 + 0.5 * thrEff,
      gain: Math.max(0, gain) * cutK,
      bright: 800 + 4500 * thrEff + rpmK * 1500,
      noiseGain: 0.02 + 0.10 * thrEff,
      slipGain: slip * 0.3,
      gear: this.gear, rpm: this.rpm,
      events: events || []
    };
  }
}
window.EngineAudioModel = EngineAudioModel;

/* ── Web Audio Sink（无 AudioContext 时 inert，沙箱/测试安全）───────── — Web Audio sink (inert without AudioContext; sandbox/test-safe) ───── */
class EngineAudioSink {
  constructor() {
    this.ok = false; this.ctx = null; this.muted = true;
    this._noiseBuf = null;
    this._lastPopT = 0;
  }

  unlock() {
    if (this.ok) { if (this.ctx.state === "suspended") this.ctx.resume(); return true; }
    const AC = (typeof window !== "undefined") && (window.AudioContext || window.webkitAudioContext);
    if (!AC) return false;
    try {
      const ctx = new AC();
      const mk = (type) => { const o = ctx.createOscillator(); o.type = type; return o; };
      
      // 1. Multi-Harmonic Combustion Oscillators (多重气缸谐波发声体 — multi-cylinder harmonic sound sources)
      this.o1 = mk("sawtooth"); // 气缸次谐波咕噜喉音 (0.5 fFiring) — sub-harmonic cylinder growl (0.5 fFiring)
      this.o2 = mk("sawtooth"); // 气缸主点火燃烧冲击 (1.0 fFiring) — main ignition combustion pulse (1.0 fFiring)
      this.o3 = mk("sawtooth"); // 二次谐波排气歧管吼声 (2.0 fFiring) — second-harmonic manifold roar (2.0 fFiring)
      this.o4 = mk("sawtooth"); // 高阶气门高转啸叫 (3.0 fFiring) — high-order valvetrain shriek at high rpm (3.0 fFiring)
      this.oTurbo = mk("sine"); // 涡轮增压高频哨音 (2.5kHz - 7kHz) — turbo high-frequency whistle (2.5kHz - 7kHz)

      this.g1 = ctx.createGain(); this.g1.gain.value = 0.85;
      this.g2 = ctx.createGain(); this.g2.gain.value = 1.0;
      this.g3 = ctx.createGain(); this.g3.gain.value = 0.65;
      this.g4 = ctx.createGain(); this.g4.gain.value = 0.35;
      this.gTurbo = ctx.createGain(); this.gTurbo.gain.value = 0.0;

      // 2. 非对称波形塑形失真 (模拟真实气缸燃烧膨胀非对称压力波) — 2. asymmetric waveshaping distortion (emulates the asymmetric pressure wave of real combustion expansion)
      this.shaper = ctx.createWaveShaper();
      this._setDrive(2.8);

      // 3. 仿生双腔体排气共振共鸣器 (Acoustic Formant Resonators) — 3. bio-inspired multi-chamber exhaust resonators (Acoustic Formant Resonators)
      // 共鸣 1: 缸体与膨胀箱低频共振腔 (185Hz, Q=2.2, +5dB) — resonator 1: block/expansion-chamber low-frequency cavity (185Hz, Q=2.2, +5dB)
      this.fBody = ctx.createBiquadFilter();
      this.fBody.type = "peaking";
      this.fBody.frequency.value = 185;
      this.fBody.Q.value = 2.2;
      this.fBody.gain.value = 5.0;

      // 共鸣 2: 排气歧管金属咆哮 (720Hz, Q=2.6, +6dB) — resonator 2: metallic manifold roar (720Hz, Q=2.6, +6dB)
      this.fBark = ctx.createBiquadFilter();
      this.fBark.type = "peaking";
      this.fBark.frequency.value = 720;
      this.fBark.Q.value = 2.6;
      this.fBark.gain.value = 6.0;

      // 共鸣 3: 尾喉金属撕裂破音 (2400Hz, Q=3.0, +4dB) — resonator 3: tailpipe metallic rasp (2400Hz, Q=3.0, +4dB)
      this.fRasp = ctx.createBiquadFilter();
      this.fRasp.type = "peaking";
      this.fRasp.frequency.value = 2400;
      this.fRasp.Q.value = 3.0;
      this.fRasp.gain.value = 4.0;

      // 动态声学低通辐射滤波 — dynamic acoustic low-pass radiation filter
      this.lp = ctx.createBiquadFilter();
      this.lp.type = "lowpass";
      this.lp.frequency.value = 1200;

      // 4. 排气湍流喷射噪声层 (Exhaust Jet Turbulence) — 4. exhaust jet turbulence noise layer (Exhaust Jet Turbulence)
      this._noiseBuf = this._makeNoise(ctx, 2.0);
      this.nSrc = ctx.createBufferSource(); this.nSrc.buffer = this._noiseBuf; this.nSrc.loop = true;
      this.nBP = ctx.createBiquadFilter(); this.nBP.type = "bandpass"; this.nBP.Q.value = 1.2;
      this.nBP.frequency.value = 850;
      this.nGain = ctx.createGain(); this.nGain.gain.value = 0.0;

      // 5. 轮胎抓地力极限高频啸叫层 (Tire Slip Squeal) — 5. tire-at-grip-limit high-frequency squeal layer (Tire Slip Squeal)
      this.sSrc = ctx.createBufferSource(); this.sSrc.buffer = this._noiseBuf; this.sSrc.loop = true;
      this.sBP = ctx.createBiquadFilter(); this.sBP.type = "bandpass"; this.sBP.Q.value = 3.5;
      this.sBP.frequency.value = 2300;
      this.sGain = ctx.createGain(); this.sGain.gain.value = 0.0;

      // 6. 主音量控制与母带动态压限器 (Dynamics Compressor & Limiter) — 6. master volume + bus dynamics compressor/limiter (Dynamics Compressor & Limiter)
      this.master = ctx.createGain(); this.master.gain.value = 0.0;
      this.comp = ctx.createDynamicsCompressor();
      this.comp.threshold.value = -10;
      this.comp.knee.value = 12;
      this.comp.ratio.value = 5;
      this.comp.attack.value = 0.003;
      this.comp.release.value = 0.12;

      // 拓扑路由连接 — topological routing connections
      this.o1.connect(this.g1); this.g1.connect(this.shaper);
      this.o2.connect(this.g2); this.g2.connect(this.shaper);
      this.o3.connect(this.g3); this.g3.connect(this.shaper);
      this.o4.connect(this.g4); this.g4.connect(this.shaper);

      this.shaper.connect(this.fBody);
      this.fBody.connect(this.fBark);
      this.fBark.connect(this.fRasp);
      this.fRasp.connect(this.lp);
      this.lp.connect(this.master);

      this.oTurbo.connect(this.gTurbo);
      this.gTurbo.connect(this.master);

      this.nSrc.connect(this.nBP); this.nBP.connect(this.nGain); this.nGain.connect(this.master);
      this.sSrc.connect(this.sBP); this.sBP.connect(this.sGain); this.sGain.connect(this.master);

      this.master.connect(this.comp);
      this.comp.connect(ctx.destination);

      this.o1.start(); this.o2.start(); this.o3.start(); this.o4.start(); this.oTurbo.start();
      this.nSrc.start(); this.sSrc.start();
      this.ctx = ctx; this.ok = true;
      return true;
    } catch (e) { return false; }
  }

  _makeNoise(ctx, sec) {
    const buf = ctx.createBuffer(1, Math.floor(ctx.sampleRate * sec), ctx.sampleRate);
    const d = buf.getChannelData(0);
    for (let i = 0; i < d.length; i++) d[i] = Math.random() * 2 - 1;
    return buf;
  }

  _setDrive(k) {
    const n = 256, curve = new Float32Array(n);
    for (let i = 0; i < n; i++) {
      const x = (i / (n - 1)) * 2 - 1;
      // 非对称软饱和传递曲线：增强偶次谐波，赋予大排量 V8 / F1 温暖且暴力的金属厚重感 — asymmetric soft-saturation transfer curve: boosts even-order harmonics, giving big-displacement V8 / F1 a warm yet violent metallic weight
      curve[i] = (x > 0)
        ? Math.tanh(k * x) / Math.tanh(k)
        : Math.tanh(k * 1.35 * x) / Math.tanh(k * 1.35);
    }
    this.shaper.curve = curve;
  }

  /* p: model 输出的声学参数帧 — p: the acoustic parameter frame output by the model */
  update(p) {
    if (!this.ok) return;
    const t = this.ctx.currentTime;
    const S = (param, v, tau) => param.setTargetAtTime(v, t, tau);

    // 1. 点火脉冲基频追踪 — 1. firing-pulse fundamental tracking
    S(this.o1.frequency, Math.max(10, p.f1), 0.02);
    S(this.o2.frequency, Math.max(15, p.f2), 0.02);
    S(this.o3.frequency, Math.max(20, p.f3), 0.02);
    S(this.o4.frequency, Math.max(30, p.f2 * 3.0), 0.02);

    // 2. 谐波配比与泛音平衡 — 2. harmonic mix and overtone balance
    S(this.g1.gain, p.h1 * 0.85, 0.03);
    S(this.g2.gain, 1.0, 0.03);
    S(this.g3.gain, p.h2 * 0.75, 0.03);
    S(this.g4.gain, p.h3 * 0.45, 0.03);

    // 3. 动态声学辐射滤波与共振激发 — 3. dynamic acoustic radiation filter and resonance excitation
    const targetLp = Math.max(400, Math.min(9500, p.bright * 1.25));
    S(this.lp.frequency, targetLp, 0.03);

    const loadFactor = Math.min(1.0, Math.max(0, (p.bright - 800) / 4500));
    this._setDrive(2.2 + 3.0 * loadFactor);
    S(this.fBark.gain, 4.0 + 4.5 * loadFactor, 0.04);
    S(this.fRasp.gain, 2.0 + 5.0 * loadFactor, 0.04);

    // 4. 涡轮增压哨音 (随节气门负荷增压) — 4. turbo whistle (boost follows throttle load)
    const turboFreq = 2600 + 4200 * loadFactor;
    S(this.oTurbo.frequency, turboFreq, 0.06);
    S(this.gTurbo.gain, this.muted ? 0 : (0.015 + 0.04 * loadFactor * loadFactor), 0.05);

    // 5. 排气喷流噪声与轮胎抓地尖叫 — 5. exhaust jet noise and tire-grip squeal
    S(this.nBP.frequency, Math.max(300, Math.min(4800, p.bright * 0.65)), 0.05);
    S(this.nGain.gain, this.muted ? 0 : p.noiseGain * 0.8, 0.03);
    S(this.sGain.gain, this.muted ? 0 : p.slipGain * 0.4, 0.05);

    // 6. 主通道音量增益 — 6. master channel volume gain
    const targetMaster = this.muted ? 0 : Math.min(0.7, p.gain * 0.55);
    S(this.master.gain, targetMaster, 0.03);
    this._lastP = p;

    // 7. 降挡回火与收油爆震放炮 — 7. downshift backfire and lift-off detonation pops
    for (const ev of p.events) {
      if (ev === "pop") this.pop();
    }
  }

  /* 真实多级回火爆震放炮：超低音爆压冲击 + 锋利金属破音 + 延迟余响 — realistic multi-stage backfire crackle: sub-bass blast impact + sharp metallic crack + delayed tail */
  pop() {
    if (!this.ok || this.muted) return;
    const ctx = this.ctx, t = ctx.currentTime;
    if (t - this._lastPopT < 0.16) return;
    this._lastPopT = t;

    // 脉冲 1: 强烈排气回火超低音轰击 (140Hz -> 38Hz 爆破下潜) — pulse 1: strong exhaust-backfire sub-bass thump (140Hz -> 38Hz dive)
    const o1 = ctx.createOscillator(); o1.type = "triangle";
    o1.frequency.setValueAtTime(140, t);
    o1.frequency.exponentialRampToValueAtTime(38, t + 0.08);
    const g1 = ctx.createGain();
    g1.gain.setValueAtTime(0.55, t);
    g1.gain.exponentialRampToValueAtTime(0.001, t + 0.09);
    o1.connect(g1); g1.connect(this.comp);
    o1.start(t); o1.stop(t + 0.10);

    // 脉冲 2: 尖锐金属破裂噪声微火花 (1.1kHz 高通瞬态切片) — pulse 2: sharp metallic crack micro-spark (1.1kHz high-pass transient slice)
    if (this._noiseBuf) {
      const nSrc = ctx.createBufferSource(); nSrc.buffer = this._noiseBuf;
      const nFilt = ctx.createBiquadFilter(); nFilt.type = "highpass"; nFilt.frequency.value = 1100;
      const ng = ctx.createGain();
      ng.gain.setValueAtTime(0.40, t);
      ng.gain.exponentialRampToValueAtTime(0.001, t + 0.07);
      nSrc.connect(nFilt); nFilt.connect(ng); ng.connect(this.comp);
      nSrc.start(t); nSrc.stop(t + 0.08);
    }

    // 脉冲 3: 滞后 50ms 的回火余响二次放炮 (连珠炮 pop-pop 质感) — pulse 3: 50 ms-delayed after-fire secondary pop (machine-gun pop-pop texture)
    const o2 = ctx.createOscillator(); o2.type = "sawtooth";
    o2.frequency.setValueAtTime(180, t + 0.05);
    o2.frequency.exponentialRampToValueAtTime(50, t + 0.12);
    const g2 = ctx.createGain();
    g2.gain.setValueAtTime(0.0, t);
    g2.gain.setValueAtTime(0.28, t + 0.05);
    g2.gain.exponentialRampToValueAtTime(0.001, t + 0.13);
    o2.connect(g2); g2.connect(this.comp);
    o2.start(t + 0.05); o2.stop(t + 0.14);
  }

  setMuted(m) {
    this.muted = m;
    if (this.ok) this.master.gain.setTargetAtTime(m ? 0 : (this._lastP ? this._lastP.gain * 0.55 : 0.1),
      this.ctx.currentTime, 0.05);
  }
}

/* ── 单例门面：三舞台 loop 每帧喂 update(dt, st, tel, ctrl) ─────────── — singleton facade: the three stage loops feed update(dt, st, tel, ctrl) every frame ──────
   watchdog：>250ms 无喂（舞台切走/暂停）自动淡出，不留空转轰鸣。 — watchdog: >250 ms without feeding (stage switched away/paused) auto-fades, leaving no idling roar.
   开关状态常驻：localStorage 记住；仅赛道舞台显示按钮（心跳控制显隐）。 — the on/off state persists: remembered in localStorage; the button is shown only on the circuit stage (heartbeat controls visibility). */
const ENGINE_SOUND_STORE_KEY = "labsus-sound-muted";
const EngineSound = {
  model: new EngineAudioModel("gt3"),
  sink: new EngineAudioSink(),
  muted: (function () {
    try { return localStorage.getItem(ENGINE_SOUND_STORE_KEY) !== "0"; } catch (e) { return true; }
  })(),
  lastFeed: 0,

  ensure(vehicleType) {
    if (this.model.profileKey !== vehicleType && ENGINE_SOUND_PROFILES[vehicleType]) {
      const wasMuted = this.muted;
      this.model = new EngineAudioModel(vehicleType);
      this.muted = wasMuted;
    }
  },

  update(dt, st, tel, ctrl) {
    const now = (typeof performance !== "undefined") ? performance.now() : Date.now();
    this.lastFeed = now;
    /* 车型联动：S 为全局裸名（浏览器=window.S）；vm 沙箱/严格环境只有 window.S，
       两者都查（缺一会导致声线永远锁 gt3） — vehicle-type coupling: S is a bare global (browser: window.S); the vm sandbox / strict environment only has window.S,
       so both are checked (missing either locks the sound profile to gt3 forever) */
    const vt = (typeof S !== "undefined" && S && S.vehicleType)
      || (typeof window !== "undefined" && window.S && window.S.vehicleType) || "gt3";
    this.ensure(vt);
    const kappa = tel && tel.kappa
      ? Math.max(Math.abs(tel.kappa.FL || 0), Math.abs(tel.kappa.FR || 0),
                 Math.abs(tel.kappa.RL || 0), Math.abs(tel.kappa.RR || 0)) : 0;
    const isKerb = !!(tel && tel.isKerb && (tel.isKerb.FL || tel.isKerb.FR || tel.isKerb.RL || tel.isKerb.RR));
    /* M-1(G31-P6-fix)：驱动轮 omega 均值 → 挡位/RPM 链。架构感知：EV+FWD 用前轮均值
       （直驱前电机），其余（legacy/RWD/AWD 及 EV+RWD/AWD/TV）沿用后轮均值。
       非 EV 架构下该值仅用于回退估算（ptRpm=null 分支），语义不变。
       — M-1(G31-P6-fix): driven-wheel omega mean → gear/RPM chain. Architecture-aware: EV+FWD uses the front-wheel mean
       (direct-drive front motor), everything else (legacy/RWD/AWD and EV+RWD/AWD/TV) keeps the rear-wheel mean.
       Under non-EV architectures this value only feeds the fallback estimate (ptRpm=null branch); semantics unchanged. */
    let wheelOmega = 0;
    if (st && st.omega) {
      const isEvFwd = (typeof POWERTRAIN !== "undefined" && POWERTRAIN.spec &&
        POWERTRAIN.spec.architecture === "ev" && POWERTRAIN.spec.drive === "fwd");
      wheelOmega = isEvFwd
        ? ((st.omega.FL || 0) + (st.omega.FR || 0)) / 2
        : ((st.omega.RL || 0) + (st.omega.RR || 0)) / 2;
    }
    /* G31-P10-3：POWERTRAIN.step() 回传帧 { iceRpm, shiftRpm, gearIdx, soc, ... }
       由接线层 11-stages.js 存于 window.physicsEngine._ptOut——直接读，最小侵入
       （不改三舞台 loop 对 EngineSound.update 的调用签名）。无 physicsEngine
       （vm 沙箱/加载早期）→ null → EngineAudioModel 内部按判据回退。
       — G31-P10-3: the frame returned by POWERTRAIN.step() { iceRpm, shiftRpm, gearIdx, soc, ... }
       is stored by the wiring layer 11-stages.js in window.physicsEngine._ptOut — read directly, minimal intrusion
       (the three stage loops\' EngineSound.update call signature unchanged). Without physicsEngine
       (vm sandbox / early load) → null → EngineAudioModel falls back internally per its gate. */
    const ptOut = (typeof window !== "undefined" && window.physicsEngine && window.physicsEngine._ptOut)
      ? window.physicsEngine._ptOut : null;
    const p = this.model.update(dt, {
      wheelOmega, throttle: (ctrl && ctrl.throttle) || 0, brake: (ctrl && ctrl.brake) || 0,
      kappaMax: kappa, isKerb, ptOut
    });
    this.sink.update(p);
    return p;
  },

  /* 用户手势入口（浏览器 autoplay 解锁）；状态持久化（刷新后不用重开） — user-gesture entry point (browser autoplay unlock); state persisted (no need to re-enable after refresh) */
  toggle() {
    this.muted = !this.muted;
    if (!this.muted) this.sink.unlock();
    this.sink.setMuted(this.muted);
    try { localStorage.setItem(ENGINE_SOUND_STORE_KEY, this.muted ? "1" : "0"); } catch (e) {}
    return !this.muted;   // true = 现在有声 — true = sound is now on
  }
};
window.EngineSound = EngineSound;

/* 上次开著声浪刷新：立即尝试解锁；AudioContext 若 suspended（无手势），
   挂一次性页面手势恢复——同源已交互站点通常自动 running
   — page refresh with sound left on: try to unlock immediately; if the AudioContext is suspended (no gesture yet),
   attach a one-shot page-gesture resume — same-origin sites already interacted with usually resume automatically */
if (!EngineSound.muted) {
  EngineSound.sink.unlock();
  const _resumeSnd = () => {
    if (EngineSound.sink.ok && EngineSound.sink.ctx.state === "suspended") EngineSound.sink.ctx.resume();
  };
  if (typeof document !== "undefined" && document.addEventListener) {
    document.addEventListener("pointerdown", _resumeSnd);
    document.addEventListener("keydown", _resumeSnd);
  }
}

/* watchdog 心跳：舞台 inactive 后 loop 停止喂帧 → 淡出为静音；
   同时控制外部按钮显隐——若为注入的浮动按钮才受此控制
   — watchdog heartbeat: once a stage is inactive its loop stops feeding → fade to silence;
   also controls the external button visibility — only the injected floating button is governed by this */
let _sndBtn = null;
if (typeof setInterval === "function") {
  setInterval(() => {
    const now = (typeof performance !== "undefined") ? performance.now() : Date.now();
    if (now - EngineSound.lastFeed > 250 && EngineSound.sink.ok) {
      EngineSound.sink.update({
        f1: 20, f2: 40, f3: 80, h1: 1, h2: 0.5, h3: 0.25,
        gain: 0, bright: 800, noiseGain: 0, slipGain: 0, events: []
      });
    }
    if (_sndBtn && _sndBtn._isCreated) {
      const inCircuit = (typeof CIRCUIT_STAGE !== "undefined" && CIRCUIT_STAGE && CIRCUIT_STAGE.active);
      _sndBtn.style.display = inCircuit ? "" : "none";
    }
  }, 120);
}

/* 声音开关按钮：优先绑定已存在于顶栏的按钮（如 circuitStageModal 内的 #engineSoundTg）；
   若不存在且页面无顶栏容器时作为后备注入。
   — sound toggle button: prefer binding the button already present in the top bar (e.g. #engineSoundTg inside circuitStageModal);
   if absent and the page has no top-bar container, inject a fallback button. */
(function _bindEngineSoundBtn() {
  const refresh = (b) => {
    b.textContent = EngineSound.muted ? "🔊 Engine sound OFF" : "🔊 Engine sound ON";
    b.style.color = EngineSound.muted ? "#8FA1B8" : "#E8A04C";
    if (b.classList) b.classList.toggle("muted", EngineSound.muted);
  };
  const ensureBtn = () => {
    let b = (typeof document !== "undefined") ? document.getElementById("engineSoundTg") : null;
    let isCreated = false;
    if (!b && typeof document !== "undefined" && document.body && document.createElement) {
      b = document.createElement("button");
      b.id = "engineSoundTg";
      b.title = "Engine sound: acoustic synthesis reacts live to engine speed / throttle / shifts / tire slip";
      b.style.cssText = "position:fixed;top:10px;right:10px;z-index:100000;padding:4px 10px;" +
        "background:#0E1520;border:1px solid #2A3646;cursor:pointer;opacity:0.85;" +
        "font:600 12px/1.4 system-ui,sans-serif;letter-spacing:0.5px;";
      b.onmouseenter = () => { b.style.opacity = "1"; };
      b.onmouseleave = () => { b.style.opacity = "0.85"; };
      document.body.appendChild(b);
      isCreated = true;
    }
    if (b && !b._sndBound) {
      b._sndBound = true;
      b.onclick = () => { EngineSound.toggle(); refresh(b); };
      refresh(b);
      _sndBtn = b;
      _sndBtn._isCreated = isCreated;
      const inCircuit0 = (typeof CIRCUIT_STAGE !== "undefined" && CIRCUIT_STAGE && CIRCUIT_STAGE.active);
      if (isCreated) b.style.display = inCircuit0 ? "" : "none";
    }
  };
  if (typeof document !== "undefined") {
    if (document.readyState === "loading" && document.addEventListener) {
      document.addEventListener("DOMContentLoaded", ensureBtn);
    } else {
      ensureBtn();
    }
  }
})();
