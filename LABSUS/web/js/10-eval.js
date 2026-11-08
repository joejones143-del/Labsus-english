"use strict";
/* 液态玻璃双主题：自动跟随系统偏好，手动切换后持久化 — liquid-glass dual theme: follows the system preference automatically, persisted after manual toggle */
function applyTheme(t){
  document.documentElement.setAttribute("data-theme",t);
  try{localStorage.setItem("labsus-theme",t);}catch(e){}
  Object.assign(C,PAL[t]);
  const tg=document.getElementById("themeTg");
  if(tg)tg.title=t==="light"?"Currently light · click to switch to dark":"Currently dark · click to switch to light";
}
(function initTheme(){
  let t=null;try{t=localStorage.getItem("labsus-theme");}catch(e){}
  if(t!=="light"&&t!=="dark")t=(window.matchMedia&&matchMedia("(prefers-color-scheme: light)").matches)?"light":"dark";
  applyTheme(t);
  const tg=document.getElementById("themeTg");
  if(tg)tg.onclick=()=>applyTheme(document.documentElement.getAttribute("data-theme")==="light"?"dark":"light");
})();

/* =====================================================================
   LABSUS 综合性能评价与AI调校诊断引擎 (Evaluation Engine)
   — LABSUS comprehensive performance-evaluation & AI tuning-diagnosis engine
   ===================================================================== */
const EVAL_BENCHMARKS = {
  camberGainF:     { name: "Front camber gain", target: "0.04 ~ 0.08 °/mm", unit: "°/mm", eval: v => (v >= 0.04 && v <= 0.08 ? 'good' : (v >= 0.02 && v <= 0.11 ? 'warn' : 'bad')), desc: "Compensates contact patch while the body rolls in corners; too small invites understeer, too large hurts braking" },
  camberGainR:     { name: "Rear camber gain", target: "0.03 ~ 0.07 °/mm", unit: "°/mm", eval: v => (v >= 0.03 && v <= 0.07 ? 'good' : (v >= 0.015 && v <= 0.09 ? 'warn' : 'bad')), desc: "Rear-axle roll grip support for high-speed stability" },
  bumpSteerF:      { name: "Front bump steer", target: "< 0.02 °/mm", unit: "°/mm", eval: v => (Math.abs(v) <= 0.02 ? 'good' : (Math.abs(v) <= 0.05 ? 'warn' : 'bad')), desc: "Directional stability over bumpy roads; too large makes the nose wander" },
  bumpSteerR:      { name: "Rear bump steer", target: "0.00 ~ +0.03 °/mm", unit: "°/mm", eval: v => (v >= 0.0 && v <= 0.03 ? 'good' : (v >= -0.01 && v <= 0.05 ? 'warn' : 'bad')), desc: "Rear toe change through bump; a slight bump-toe-in aids high-speed stability" },
  rcHeightF:       { name: "Front roll-center height", target: "30 ~ 75 mm", unit: "mm", eval: v => (v >= 30 && v <= 75 ? 'good' : (v >= 10 && v <= 100 ? 'warn' : 'bad')), desc: "Geometric transfer height of front-axle lateral force; too high risks the jacking effect" },
  rcHeightR:       { name: "Rear roll-center height", target: "45 ~ 95 mm", unit: "mm", eval: v => (v >= 45 && v <= 95 ? 'good' : (v >= 20 && v <= 120 ? 'warn' : 'bad')), desc: "Rear roll center should sit above the front one, forming a natural forward-leaning roll axis" },
  antiDiveF:       { name: "Front anti-dive ratio", target: "20% ~ 40%", unit: "%", eval: v => (v >= 20 && v <= 40 ? 'good' : (v >= 10 && v <= 55 ? 'warn' : 'bad')), desc: "Front suspension anti-compression geometry under braking; suppresses pitch" },
  antiSquatR:      { name: "Rear anti-squat ratio", target: "30% ~ 60%", unit: "%", eval: v => (v >= 30 && v <= 60 ? 'good' : (v >= 15 && v <= 80 ? 'warn' : 'bad')), desc: "Rear suspension anti-compression geometry under acceleration; prevents CoG squat" },
  scrubRadiusF:    { name: "Front scrub radius", target: "5 ~ 25 mm", unit: "mm", eval: v => (v >= 5 && v <= 25 ? 'good' : (v >= -5 && v <= 40 ? 'warn' : 'bad')), desc: "Steering-wheel feel feedback and tolerance to brake pull" },
  casterTrailF:    { name: "Front caster trail", target: "15 ~ 35 mm", unit: "mm", eval: v => (v >= 15 && v <= 35 ? 'good' : (v >= 5 && v <= 45 ? 'warn' : 'bad')), desc: "Lever arm of the tire self-aligning torque; too large makes steering heavy" },
  motionRatioF:    { name: "Front motion ratio", target: "0.65 ~ 0.85", unit: "", eval: v => (v >= 0.65 && v <= 0.85 ? 'good' : (v >= 0.5 && v <= 0.95 ? 'warn' : 'bad')), desc: "Spring/damper lever efficiency; near-linear is recommended" },
  motionRatioR:    { name: "Rear motion ratio", target: "0.70 ~ 0.90", unit: "", eval: v => (v >= 0.70 && v <= 0.90 ? 'good' : (v >= 0.55 && v <= 1.0 ? 'warn' : 'bad')), desc: "Rear-axle spring/damper lever efficiency" },
  wheelRateF:      { name: "Front wheel rate", target: "30 ~ 70 N/mm", unit: "N/mm", eval: v => (v >= 30 && v <= 70 ? 'good' : (v >= 20 && v <= 90 ? 'warn' : 'bad')), desc: "Front suspension vertical support stiffness at the wheel" },
  wheelRateR:      { name: "Rear wheel rate", target: "35 ~ 75 N/mm", unit: "N/mm", eval: v => (v >= 35 && v <= 75 ? 'good' : (v >= 25 && v <= 95 ? 'warn' : 'bad')), desc: "Rear suspension vertical support stiffness at the wheel" },
  rideFreqF:       { name: "Front ride frequency", target: "1.8 ~ 3.2 Hz", unit: "Hz", eval: v => (v >= 1.8 && v <= 3.2 ? 'good' : (v >= 1.4 && v <= 4.0 ? 'warn' : 'bad')), desc: "Front axle vertical natural vibration frequency" },
  rideFreqR:       { name: "Rear ride frequency", target: "2.0 ~ 3.5 Hz", unit: "Hz", eval: v => (v >= 2.0 && v <= 3.5 ? 'good' : (v >= 1.5 && v <= 4.2 ? 'warn' : 'bad')), desc: "Rear ride frequency is usually 10-15% above the front to smooth out ride harshness" },
  dampingRatioF:   { name: "Front damping ratio", target: "0.55 ~ 0.75", unit: "", eval: v => (v >= 0.55 && v <= 0.75 ? 'good' : (v >= 0.4 && v <= 0.85 ? 'warn' : 'bad')), desc: "Front axle vibration decay; prevents excess bouncing" },
  dampingRatioR:   { name: "Rear damping ratio", target: "0.60 ~ 0.80", unit: "", eval: v => (v >= 0.60 && v <= 0.80 ? 'good' : (v >= 0.45 && v <= 0.90 ? 'warn' : 'bad')), desc: "Rear axle vibration decay" },
  rollStiffnessPct:{ name: "Front roll-stiffness share (F/T)", target: "48% ~ 58%", unit: "%", eval: v => (v >= 48 && v <= 58 ? 'good' : (v >= 40 && v <= 65 ? 'warn' : 'bad')), desc: "Sets steady-state cornering balance: >55% trends understeer, <48% trends oversteer" },
  ackermannPct:    { name: "Ackermann steering %", target: "30% ~ 80%", unit: "%", eval: v => (v == null ? 'warn' : (v >= 30 && v <= 80 ? 'good' : (v >= 10 && v <= 100 ? 'warn' : 'bad'))), desc: "Race cars usually run 40-60% to reduce outer-tire drag; shows — when not computed from the steer sweep (no more fake data)" },
  usGradient:      { name: "Steady-state understeer gradient", target: "0.2 ~ 2.0 °/g", unit: "°/g", eval: v => (v == null ? 'warn' : (v >= 0.2 && v <= 2.0 ? 'good' : (v >= 0.05 && v <= 3.5 ? 'warn' : 'bad'))), desc: "The No.1 chassis-development KPI: slope of the front/rear slip-angle difference vs lateral acceleration; positive = understeer, negative = oversteer. Includes tire load sensitivity; high-mu tire sets naturally show smaller gradients (compare mu-normalized); shows — when |gy|<0.02g" }
};

function calculateEvaluationData(){
  const swF = SIM.swF, swR = SIM.swR;
  const mF = SIM.mFR, mR = SIM.mRR;
  if(!swF || !swR || !mF || !mR) return null;

  const sampleSweep = (sw, tr0) => {
    if(!sw.rows || sw.rows.length === 0) return null;
    let closest = sw.rows[0], minDiff = 1e9;
    for(const r of sw.rows){
      const diff = Math.abs(r.tr - tr0);
      if(diff < minDiff){ minDiff = diff; closest = r; }
    }
    return closest;
  };

  const nomF = sampleSweep(swF, 0) || swF.rows[Math.floor(swF.rows.length/2)];
  const nomR = sampleSweep(swR, 0) || swR.rows[Math.floor(swR.rows.length/2)];
  
  const trArrF = swF.rows.map(r => r.tr);
  const trArrR = swR.rows.map(r => r.tr);
  const dTrF = Math.max(1, trArrF[trArrF.length-1] - trArrF[0]);
  const dTrR = Math.max(1, trArrR[trArrR.length-1] - trArrR[0]);

  const camGainF = Math.abs((swF.rows[swF.rows.length-1].cam - swF.rows[0].cam) / dTrF);
  const camGainR = Math.abs((swR.rows[swR.rows.length-1].cam - swR.rows[0].cam) / dTrR);
  const bumpSteerF = Math.abs((swF.rows[swF.rows.length-1].toe - swF.rows[0].toe) / dTrF);
  const bumpSteerR = (swR.rows[swR.rows.length-1].toe - swR.rows[0].toe) / dTrR;

  const mrF = Math.abs(nomF.mr || 0.75);
  const mrR = Math.abs(nomR.mr || 0.78);
  const kwF = (S.front.kS || 60) * (mrF * mrF);
  const kwR = (S.rear.kS || 65) * (mrR * mrR);
  const cwF = (S.front.cR || 3.5) * (mrF * mrF);
  const cwR = (S.rear.cR || 4.0) * (mrR * mrR);

  /* F-17（2026-08-30）：簧载质量前后分配不再硬编码 45/55 —— 用预设
     前后轴簧载质量比例（S.front.mS / S.rear.mS，与准静态内核同源），
     换车型后偏频/阻尼比不再系统性偏差。
     — F-17: sprung-mass front/rear split is no longer hard-coded 45/55 — uses the preset front/rear sprung-mass ratios (S.front.mS / S.rear.mS, same source as the quasi-static core) so ride frequency / damping ratio no longer skew systematically after switching vehicles. */
  const msF = (S.front.mS || 55), msR = (S.rear.mS || 65);
  const mSprTot = (S.mSprung || 400);
  const mSprungF = mSprTot * (msF / (msF + msR)) / 2;   /* 单侧簧载质量 — sprung mass per side */
  const mSprungR = mSprTot * (msR / (msF + msR)) / 2;
  const freqF = (1 / (2 * Math.PI)) * Math.sqrt((kwF * 1000) / mSprungF);
  const freqR = (1 / (2 * Math.PI)) * Math.sqrt((kwR * 1000) / mSprungR);
  const ccF = 2 * Math.sqrt((kwF * 1000) * mSprungF);
  const ccR = 2 * Math.sqrt((kwR * 1000) * mSprungR);
  const dampRatioF = (cwF * 1000) / (ccF || 1);
  const dampRatioR = (cwR * 1000) / (ccR || 1);

  const tf = Math.abs(S.front.hp.WC[0] * 2);
  const tr = Math.abs(S.rear.hp.WC[0] * 2);
  const kRollF = 0.5 * (kwF * 1000) * (tf * tf / 1e6) + (S.front.arb.d > 0 ? 1200 : 0);
  const kRollR = 0.5 * (kwR * 1000) * (tr * tr / 1e6) + (S.rear.arb.d > 0 ? 900 : 0);
  const rollSplitF = (kRollF / (kRollF + kRollR || 1)) * 100;

  /* F-15（2026-08-30）：`||` 会吞掉合法 0 值（RC 恰在地面=0mm、scrub=0 等）；
     fin() 仅对 undefined/NaN 回退，0 值如实显示。
     — F-15: `||` swallows legitimate 0 values (RC exactly at ground = 0mm, scrub = 0, etc.); fin() only falls back on undefined/NaN, 0 is displayed truthfully. */
  const fin=(v,d)=>Number.isFinite(v)?v:d;
  const metrics = {
    camberGainF: { val: camGainF, formatted: camGainF.toFixed(3) },
    camberGainR: { val: camGainR, formatted: camGainR.toFixed(3) },
    bumpSteerF:  { val: bumpSteerF, formatted: bumpSteerF.toFixed(3) },
    bumpSteerR:  { val: bumpSteerR, formatted: bumpSteerR.toFixed(3) },
    rcHeightF:   { val: fin(nomF.rcH,45), formatted: fin(nomF.rcH,45).toFixed(1) },
    rcHeightR:   { val: fin(nomR.rcH,65), formatted: fin(nomR.rcH,65).toFixed(1) },
    antiDiveF:   { val: fin(nomF.anti,28), formatted: fin(nomF.anti,28).toFixed(1) },
    antiSquatR:  { val: fin(nomR.anti,42), formatted: fin(nomR.anti,42).toFixed(1) },
    scrubRadiusF:{ val: fin(mF.scrub,15), formatted: fin(mF.scrub,15).toFixed(1) },
    casterTrailF:{ val: fin(mF.trail,22), formatted: fin(mF.trail,22).toFixed(1) },
    motionRatioF:{ val: mrF, formatted: mrF.toFixed(3) },
    motionRatioR:{ val: mrR, formatted: mrR.toFixed(3) },
    wheelRateF:  { val: kwF, formatted: kwF.toFixed(1) },
    wheelRateR:  { val: kwR, formatted: kwR.toFixed(1) },
    rideFreqF:   { val: freqF, formatted: freqF.toFixed(2) },
    rideFreqR:   { val: freqR, formatted: freqR.toFixed(2) },
    dampingRatioF:{ val: dampRatioF, formatted: dampRatioF.toFixed(2) },
    dampingRatioR:{ val: dampRatioR, formatted: dampRatioR.toFixed(2) },
    rollStiffnessPct: { val: rollSplitF, formatted: rollSplitF.toFixed(1) },
    /* F-11（2026-08-30）：`S.ackermann || 52` 是假数据——S.ackermann 全文无赋值点，
       恒 52% 使红绿灯失真。未计算时如实显示 "—"（eval 侧同步处理 null）。
       — F-11: `S.ackermann || 52` was fake data — S.ackermann has no assignment anywhere, so it was permanently 52% and skewed the traffic lights. When not computed, show "—" truthfully (eval side also handles null). */
    ackermannPct:{ val: Number.isFinite(S.ackermann) ? S.ackermann : null,
                   formatted: Number.isFinite(S.ackermann) ? S.ackermann.toFixed(0) : "—" },
    /* 稳态不足转向梯度：取当前准静态工况（需 |gy|≥0.02g）；镜像引擎
       chassis.quasi_loads 的轮胎分量公式（含 LS 载荷敏感性）。
       — Steady-state understeer gradient: taken from the current quasi-static load case (requires |gy| >= 0.02g); mirrors the engine chassis.quasi_loads tire-component formula (including LS load sensitivity). */
    usGradient:{ val: (SIM.qsRes && Math.abs(S.qs.gy) >= 0.02 && Number.isFinite(SIM.qsRes.usGrad)
                       && Math.abs(SIM.qsRes.usGrad) > 1e-9) ? SIM.qsRes.usGrad : null,
                 formatted: (SIM.qsRes && Math.abs(S.qs.gy) >= 0.02 && Number.isFinite(SIM.qsRes.usGrad)
                       && Math.abs(SIM.qsRes.usGrad) > 1e-9) ? SIM.qsRes.usGrad.toFixed(2) : "—" }
  };

  /* F-18（2026-08-30）：dimScores 与 EVAL_BENCHMARKS 双轨制消除 —— 旧公式
     与右侧交通灯评级独立定义（雷达图可给 good 分数、交通灯同值却 warn）。
     改为复用 EVAL_BENCHMARKS 的 eval() 评级（good=90 / warn=62 / bad=30），
     雷达图与交通灯永远同源、不可能矛盾。
     — F-18: the dimScores / EVAL_BENCHMARKS double-track system is removed — the old formula and the right-side traffic-light rating were defined independently (radar could score good while the same value rated warn on the traffic light). Now the eval() rating from EVAL_BENCHMARKS is reused (good=90 / warn=62 / bad=30), so the radar and traffic lights share one source and can never contradict. */
  const sc=(bm,val)=>{ if(val==null)return 30;
    const r=(bm&&bm.eval)?bm.eval(val):null;
    return r==="good"?90:(r==="warn"?62:30); };
  const dimScores = {
    "Camber grip": sc(EVAL_BENCHMARKS.camberGainF, camGainF),
    "Bump steer": sc(EVAL_BENCHMARKS.bumpSteerF, bumpSteerF),
    "Roll center": sc(EVAL_BENCHMARKS.rcHeightF, nomF.rcH),
    "Anti-dive pitch": sc(EVAL_BENCHMARKS.antiDiveF, nomF.anti),
    "Ride damping": sc(EVAL_BENCHMARKS.dampingRatioF, dampRatioF),
    "Steering feel": sc(EVAL_BENCHMARKS.scrubRadiusF, mF.scrub)
  };

  let totalScore = 0, count = 0;
  for(const k in dimScores){ totalScore += dimScores[k]; count++; }
  const overall = Math.round(totalScore / count);

  return { metrics, dimScores, overall };
}
window.calculateEvaluationData = calculateEvaluationData;
window.openSuspensionEvaluation = openSuspensionEvaluation;
/* G14（2026-08-31）F-61：此处原有一条把 openSlopeStage 挂到 window 的导出语句。
   该函数定义在 11-stages.js（本文件之后的脚本），G9 拆分为多个 <script src>
   之后函数不再跨文件提升 → 那行抛 ReferenceError → 本文件在此中止执行 →
   下方的 class TrackPath / CirclePath / StraightPath / CircuitPath 停留在
   TDZ 未初始化 → openSlopeStage() 里 new StraightPath(...) 抛错 →
   requestAnimationFrame(slopeStageLoop) 永不执行 = 打开舞台即黑屏。
   11-stages.js 末尾已有正确的导出，此处不得再重复导出。
   — G14 (2026-08-31) F-61: this spot used to hold a statement exporting openSlopeStage onto window. That function is defined in 11-stages.js (a script that loads AFTER this file); after G9 split the bundle into multiple <script src> tags, functions no longer hoist across files -> that line threw ReferenceError -> this file aborted here -> the TrackPath / CirclePath / StraightPath / CircuitPath classes below stayed uninitialized in TDZ -> new StraightPath(...) inside openSlopeStage() threw -> requestAnimationFrame(slopeStageLoop) never ran = black screen on opening the stage. The correct export already exists at the end of 11-stages.js; do not re-export here. */

function openSuspensionEvaluation(){
  const modal = document.getElementById("suspEvalModal");
  if(!modal) return;
  const data = calculateEvaluationData();
  if(!data) return;

  document.getElementById("evalVehSubtitle").textContent = (VEHICLE_PRESETS[S.vehicleType]||{}).name || "FIA GT3";
  document.getElementById("evalOverallScore").textContent = data.overall;
  
  const badge = document.getElementById("evalGradeBadge");
  if(data.overall >= 90){ badge.textContent = "GRADE S+ Elite competition"; badge.style.background = "#238636"; }
  else if(data.overall >= 80){ badge.textContent = "GRADE A Excellent tuning"; badge.style.background = "#1f6feb"; }
  else if(data.overall >= 70){ badge.textContent = "GRADE B Good balance"; badge.style.background = "#d29922"; }
  else { badge.textContent = "GRADE C Defects present"; badge.style.background = "#da3633"; }

  const scoreGrid = document.getElementById("evalDimensionScores");
  scoreGrid.innerHTML = "";
  for(const k in data.dimScores){
    const d = document.createElement("div");
    d.className = "eval-score-item";
    d.innerHTML = `<span>${k}</span><b>${Math.round(data.dimScores[k])} pts</b>`;
    scoreGrid.appendChild(d);
  }

  const adviceList = document.getElementById("evalAdviceList");
  adviceList.innerHTML = "";
  const advices = [];
  if(data.metrics.camberGainF.val < 0.035) advices.push({ type: 'danger', text: "[Front camber gain too low] Currently only " + data.metrics.camberGainF.formatted + " °/mm — front tire contact degrades as the body rolls in corners, inviting violent understeer! Lengthen the lower control arm or raise the outer ball joint of the upper control arm." });
  if(data.metrics.bumpSteerF.val > 0.03) advices.push({ type: 'danger', text: "[Front bump steer too large] Currently " + data.metrics.bumpSteerF.formatted + " °/mm — the wheel will fight the driver hard over kerbs! Adjust steering rack height (RACK_Z) so the tie rod coincides with the lower control arm's virtual instant center." });
  if(data.metrics.rcHeightF.val < 20) advices.push({ type: 'warn', text: "[Front roll center too low] The oversized roll moment arm makes the body roll hard in corners; adjust the inner/outer ball-joint heights appropriately." });
  if(data.metrics.rollStiffnessPct.val < 48) advices.push({ type: 'warn', text: "[Front roll-stiffness share too low] The rear axle is currently too stiff — rear grip breaks away on high-speed corner exit and the car trends to oversteer." });
  if(advices.length === 0) advices.push({ type: 'good', text: "[Excellent geometry] Every suspension metric currently sits inside the pro-racing golden window; camber compensation and bump steer are extremely well converged!" });

  advices.forEach(adv => {
    const el = document.createElement("div");
    el.className = `eval-advice-item ${adv.type}`;
    el.textContent = adv.text;
    adviceList.appendChild(el);
  });

  const grid = document.getElementById("evalMetricsGrid");
  grid.innerHTML = "";
  for(const k in data.metrics){
    const def = EVAL_BENCHMARKS[k];
    if(!def) continue;
    const m = data.metrics[k];
    const status = def.eval(m.val);
    const card = document.createElement("div");
    card.className = "metric-card";
    card.innerHTML = `
      <div class="metric-card-top">
        <span class="metric-name">${def.name}</span>
        <span class="metric-status-dot ${status === 'good' ? 'green' : (status === 'warn' ? 'yellow' : 'red')}"></span>
      </div>
      <div class="metric-val-row">
        <span class="metric-val">${m.formatted} <small style="font-size:10px;font-weight:normal;">${def.unit}</small></span>
        <span class="metric-target">Target: ${def.target}</span>
      </div>
      <div class="metric-desc">${def.desc}</div>
    `;
    grid.appendChild(card);
  }

  drawRadarChart(data.dimScores);
  modal.classList.add("show");
  /* M9（T1-3）：导出按钮在每次打开报告时重绑，避免与关闭按钮的 handler 顺序耦合 — M9 (T1-3): the export button is re-bound every time the report opens, avoiding handler-order coupling with the close button */
  const eb = document.getElementById("evalExportBtn");
  if(eb) eb.onclick = exportDesignReport;
}

/* ═══════════════════════════════════════════════════════════════════════════
   M9（2026-09-20）· T1-3 一键设计报告导出（EXPORT DESIGN REPORT）
   ───────────────────────────────────────────────────────────────────────────
   比赛设计报告是刚需，而此前只能靠截图拼。这里把**当前状态**直接序列化成
   一份自包含、可打印的 HTML：
     车辆参数 · 综合评分与六维雷达 · 关键 KPI · 四视图快照 · 特性曲线 ·
     操稳读数 · 敏感度扫描结果（若有）· 基准差异（若有）· 求解器状态与免责说明
   设计约束：
   1. **零依赖**：不引 jsPDF / 不联网，纯字符串 + canvas.toDataURL（与项目
      "零构建、离线可用"一致）。打印用浏览器自带（Ctrl+P → 另存 PDF）。
   2. **数据同源**：KPI 全部走 calculateEvaluationData()（与屏幕上的评价报告
      同一个函数），敏感度走 SIM.sensRes（扫描引擎写的那份），视图与曲线直接
      取画布像素——不另算一套，报告与屏幕不可能对不上。
   3. **诚实标注**：报告末尾显式写清模型边界（准静态线性区、载荷敏感性参考、
      未含项），避免拿去做超出模型的结论——本项目的红线。
   ── M9 2026-09-20 - T1-3 one-click design-report export. Competition design reports are a hard requirement and previously only screenshots could be stitched together; this serializes the CURRENT STATE directly into a self-contained, printable HTML: vehicle parameters - overall score & six-dimension radar - key KPIs - four-view snapshots - characteristic curves - handling readouts - sensitivity-sweep results (if any) - baseline delta (if any) - solver status & disclaimers. Design constraints: 1. zero dependencies: no jsPDF / no network, pure strings + canvas.toDataURL (consistent with the project's zero-build, offline-usable policy); printing uses the browser's own flow (Ctrl+P -> save as PDF). 2. single data source: all KPIs go through calculateEvaluationData() (the same function as the on-screen evaluation report), sensitivity uses SIM.sensRes (the copy written by the sweep engine), views and curves are taken directly from canvas pixels — nothing is recomputed, so the report and the screen can never disagree. 3. honest labeling: the end of the report explicitly states the model boundaries (quasi-static linear region, load-sensitivity reference, exclusions), to avoid conclusions beyond the model — this project's red line.
   ═══════════════════════════════════════════════════════════════════════════ */
function exportDesignReport(){
  let data = null;
  try{ data = (typeof calculateEvaluationData === "function") ? calculateEvaluationData() : null; }catch(e){ data = null; }
  const esc = s => String(s == null ? "" : s)
    .replace(/&/g,"&amp;").replace(/</g,"&lt;").replace(/>/g,"&gt;");
  const num = (v, d) => (typeof v === "number" && isFinite(v)) ? v.toFixed(d === undefined ? 2 : d) : "—";

  /* 车辆与参数 — vehicle & parameters */
  const preset = (typeof VEHICLE_PRESETS !== "undefined" && VEHICLE_PRESETS[S.vehicleType]) || {};
  const vehName = preset.name || S.vehicleType;
  const tireF = (S.front && S.front.tire) || {}, tireR = (S.rear && S.rear.tire) || {};
  const tq = (typeof TIRE_MF_QS !== "undefined") ? TIRE_MF_QS : {};
  const fzRef = Math.max(1, (S.mTotal || 0) * 9.81 / 4);

  const vehRows = [
    ["Vehicle platform", vehName],
    ["Wheelbase wb", num(S.wb, 0) + " mm"],
    ["Total mass", num(S.mTotal, 0) + " kg"],
    ["Total sprung mass", num(S.mSprung, 0) + " kg"],
    ["CoG height hcg / sprung CoG height hs", num(S.hcg, 0) + " / " + num(S.hs, 0) + " mm"],
    ["Front/rear corner sprung mass", num(S.front.mS, 1) + " / " + num(S.rear.mS, 1) + " kg"],
    ["Front/rear unsprung (per side)", num(S.front.mU, 1) + " / " + num(S.rear.mU, 1) + " kg"],
    ["Front/rear main spring rate", num(S.front.kS, 0) + " / " + num(S.rear.kS, 0) + " N/mm"],
    ["Front/rear tire vertical stiffness", num(S.front.kT, 0) + " / " + num(S.rear.kT, 0) + " N/mm"],
    ["Front/rear ARB diameter", num(S.front.arb && S.front.arb.d, 1) + " / " + num(S.rear.arb && S.rear.arb.d, 1) + " mm"],
    ["Front/rear tire width", num(tireF.W, 0) + " / " + num(tireR.W, 0) + " mm"],
    ["Tire MF", "Fy0=" + num(tq.Fy0, 0) + " N · FzNom=" + num(tq.FzNom, 0) + " N · By=" + num(tq.By, 1)
              + " · Cy=" + num(tq.Cy, 2) + " · LS=" + num(tq.LS, 3) + " · FzRef=" + num(fzRef, 0) + " N"],
    ["Quasi-static load case", "gy=" + num(S.qs && S.qs.gy, 2) + "g · gx=" + num(S.qs && S.qs.gx, 2)
                 + "g · Aero downforce " + num(S.qs && S.qs.aeroF, 0) + " N"],
  ];

  /* KPI 表。两个真源，都不得另算：
     · 四轮定位角（camber/toe/KPI/caster）——右坞站读数的同一个 metrics 对象
       （`SIM.mFR/mRR`）；注意 calculateEvaluationData 的 `metrics` 里**没有**这四个键，
       它们是整车定位量而非评价维度量。
     · 其余评价类量 —— calculateEvaluationData().metrics[key].formatted
       （值为 `{val, formatted}` 结构；直接 num() 会全渲染成 "—"）。
       — KPI table. Two sources of truth, neither may be recomputed: (a) the four wheel-alignment angles (camber/toe/KPI/caster) — the same metrics object as the right-dock readouts (SIM.mFR/mRR); note calculateEvaluationData's metrics does NOT contain these four keys — they are whole-vehicle alignment quantities, not evaluation-dimension quantities. (b) all other evaluation quantities come from calculateEvaluationData().metrics[key].formatted (a {val, formatted} structure; num() on it directly would render everything as "—"). */
  const mM = ((S.axis === "front" ? SIM.mFR : SIM.mRR) || {});
  const kpi = (data && data.metrics) || {};
  const kv = k => { const o = kpi[k];
    if(o == null) return "—";
    if(typeof o === "number") return num(o, 2);
    if(o.formatted != null && o.formatted !== "") return String(o.formatted);
    return (typeof o.val === "number" && isFinite(o.val)) ? num(o.val, 2) : "—";
  };
  const kpiRows = [
    ["Wheel camber", num(mM.cam, 2) + " °"],
    ["Wheel toe angle", num(mM.toe, 3) + " °"],
    ["Kingpin inclination (KPI / SAI)", num(mM.kpi, 2) + " °"],
    ["Caster angle", num(mM.cast, 2) + " °"],
    ["Scrub radius", kv("scrubRadiusF") + " mm"],
    ["Caster trail", kv("casterTrailF") + " mm"],
    ["Camber gain", kv("camberGainF") + " °/25mm"],
    ["Bump steer", kv("bumpSteerF") + " °/25mm"],
    ["Roll center height", kv("rcHeightF") + " mm"],
    ["Anti-dive ratio", kv("antiDiveF") + " %"],
    ["Motion ratio", kv("motionRatioF")],
    ["Wheel rate", kv("wheelRateF") + " N/mm"],
    ["Ride frequency", kv("rideFreqF") + " Hz"],
    ["Damping ratio", kv("dampingRatioF")],
    ["Front roll-stiffness share", kv("rollStiffnessPct") + " %"],
    ["Ackermann %", kv("ackermannPct") + " %"],
    ["Understeer gradient (US)", kv("usGradient") + " °/g"],
  ];
  const qs = SIM.qsRes || {};
  const qsRows = [
    ["Steady-state roll angle", num(qs.rollDeg, 2) + " °"], ["Vehicle roll gradient", num(qs.rollGrad, 2) + " °/g"],
    ["Front/rear roll stiffness", num(qs.kphi_f, 0) + " / " + num(qs.kphi_r, 0) + " N·m/°"],
    ["Front/rear slip angle", num(qs.alphaF, 2) + " / " + num(qs.alphaR, 2) + " °"],
    ["TLLTD (front axle load-transfer share)", num(qs.tlltd, 1) + " %"],
    ["Four-wheel contact loads FL/FR/RL/RR", qs.fz ? [qs.fz.FL, qs.fz.FR, qs.fz.RL, qs.fz.RR].map(v=>num(v,0)).join(" / ") + " N" : "—"],
    ["Jacking rise", num(qs.jackingMm, 2) + " mm"],
  ];

  /* 画布 → 图片。
     ⚠ 捕获前必须**强制重绘一次**：打开评价模态会改变布局 → ResizeObserver 触发
     sizeView → 画布被 resize 清空，而下一帧 rAF 还没跑（同步代码里）⇒ 直接
     toDataURL 会拍到空白帧（上一版实测：视图快照是白框、曲线图正常）。
     — canvas -> image. WARNING: force one full redraw before capturing. Opening the evaluation modal changes the layout -> ResizeObserver fires -> sizeView -> the canvas is resized and cleared, while the next rAF frame has not run yet (we are inside synchronous code) -> toDataURL would capture a blank frame (measured in the previous version: view snapshots were white boxes while curve plots were fine). */
  try{ if(typeof drawAll === "function") drawAll(); }catch(e){}
  const imgs = [];
  const pushCanvas = (cv, label) => {
    if(!cv || !cv.width || cv.width < 40) return;
    let url = null;
    try{
      /* 降采样到 ≤900px 宽：视图画布是全分辨率（1238×794 @dpr2 = 493KB/张），
         直接把 dataURL 塞进报告会让单份报告涨到 750KB+；打印 900px 足够，
         并铺白底（打印不依赖透明）。曲线小图不动。
         — Downsample to <=900px width: the view canvases are full resolution (1238x794 @dpr2 = 493KB each), so embedding dataURLs directly would push a single report past 750KB; 900px is plenty for printing, and a white background is painted (printing does not rely on transparency). Small curve plots are untouched. */
      const MAXW = 900;
      if(cv.width > MAXW){
        const k = MAXW / cv.width;
        const oc = document.createElement("canvas");
        oc.width = MAXW; oc.height = Math.max(1, Math.round(cv.height * k));
        const octx = oc.getContext("2d");
        octx.fillStyle = "#ffffff"; octx.fillRect(0, 0, oc.width, oc.height);
        octx.drawImage(cv, 0, 0, oc.width, oc.height);
        url = oc.toDataURL("image/png");
      } else {
        url = cv.toDataURL("image/png");
      }
    }catch(e){ return; }
    imgs.push({ label, url });
  };
  if(typeof VW !== "undefined") VW.forEach(v => { if(v && v.w > 60) pushCanvas(v.cv, (v.zh||"") + " " + (v.en||"")); });
  if(typeof UI !== "undefined" && UI.plots) UI.plots.forEach((cv,i)=> pushCanvas(cv, "Characteristic curve " + (i+1)));
  pushCanvas(document.getElementById("usCurvePlot"), "Understeer characteristic δ-ay");
  pushCanvas(document.getElementById("evalRadarCanvas"), "Six-dimension score radar");

  /* 敏感度扫描（T1-1 结果） — sensitivity sweep (T1-1 result) */
  const sr = SIM.sensRes;
  let sensHtml = "<p class='mut'>No sensitivity sweep has been run this session (left panel: Analysis -> Sensitivity sweep).</p>";
  if(sr && sr.rows){
    const rows = sr.rows.slice().sort((a,b)=> Math.abs(b.S) - Math.abs(a.S));
    sensHtml = "<p class='mut'>Parameter: <b>" + esc(sr.paramLabel) + "</b> ±" + num(sr.delta,1) + (sr.paramUnit||"")
      + " · " + sr.n + " points · " + rows.length + " objectives · parameter restored to " + num(sr.base,2) + (sr.paramUnit||"")
      + "</p><table><thead><tr><th>Objective</th><th>Unit</th><th>Baseline</th><th>−Δ</th><th>+Δ</th><th>Slope / unit</th><th>Sensitivity</th></tr></thead><tbody>";
    rows.forEach(r=>{
      const bad = !isFinite(r.S);
      sensHtml += "<tr><td>" + esc(r.label) + "</td><td>" + esc(r.unit) + "</td><td>" + num(r.v0,2) + "</td><td>"
        + num(r.vLo,2) + "</td><td>" + num(r.vHi,2) + "</td><td>" + num(r.slope,4) + "</td><td class='"
        + (bad ? "" : (Math.abs(r.S) >= 5 ? "hot" : "")) + "'>" + (bad ? "—" : ((r.S>=0?"+":"")+r.S.toFixed(1)+"%")) + "</td></tr>";
    });
    sensHtml += "</tbody></table>";
  }

  /* 双参数交互（E12 结果）：网格表 + 交互强度判定。
     阈值判定走 sens2Verdict() 单一真源，避免面板与报告各写一套阈值后漂移。
     — Two-parameter interaction (E12 result): grid table + interaction-strength verdict. The threshold verdict goes through the single sens2Verdict() source, so the panel and the report never drift apart with two sets of thresholds. */
  const s2 = SIM.sens2Res;
  let inter2Html = "<p class='mut'>No two-parameter interaction sweep has been run this session (left panel: Analysis -> Two-parameter interaction sweep).</p>";
  if(s2 && s2.M && s2.M.length){
    const verdict = (typeof sens2Verdict === "function") ? sens2Verdict(s2.rel) : "";
    inter2Html = "<p class='mut'>X: <b>" + esc(s2.xLabel) + "</b> ±" + num(s2.dx,1) + esc(s2.xUnit)
      + " · Y: <b>" + esc(s2.yLabel) + "</b> ±" + num(s2.dy,1) + esc(s2.yUnit)
      + " · " + s2.n + "×" + s2.n + " grid · objective: " + esc(s2.objLabel) + " [" + esc(s2.objUnit) + "]</p>"
      + "<table><thead><tr><th>Y ↓ / X →</th>"
      + s2.xs.map(v=>"<th>" + num(v,1) + "</th>").join("") + "</tr></thead><tbody>";
    for(let j = s2.n-1; j >= 0; j--){
      inter2Html += "<tr><th>" + num(s2.ys[j],1) + "</th>"
        + s2.M[j].map(v=>"<td>" + (isFinite(v) ? num(v,3) : "—") + "</td>").join("") + "</tr>";
    }
    inter2Html += "</tbody></table>"
      + "<p>Main effect X " + (s2.mainX>=0?"+":"") + num(s2.mainX,4) + " · Y " + (s2.mainY>=0?"+":"") + num(s2.mainY,4)
      + " · Interaction term " + (s2.inter>=0?"+":"") + num(s2.inter,4)
      + " (relative strength <b>" + (s2.rel*100).toFixed(0) + "%</b>) ⇒ " + esc(verdict) + "</p>";
  }

  /* 多方案对比（E11 结果）：直接复用方案库的对比行，不另算 — multi-setup comparison (E11 result): reuse the setup library's comparison rows directly, nothing recomputed */
  let setupCmpHtml = "<p class='mut'>No setups checked (left panel: Analysis -> Setup library & comparison, check 2-3 rows).</p>";
  try{
    if(typeof setupCompareRows === "function"){
      const cmp = setupCompareRows();
      if(cmp.sel.length >= 2){
        setupCmpHtml = "<p class='mut'>"
          + cmp.sel.map((r,i)=> (i===0?"Base: ":"") + "<b>" + esc(r.name) + "</b>").join(" · ")
          + " (Δ vs baseline column)</p><table><thead><tr><th>KPI</th><th>Unit</th>"
          + cmp.sel.map(r=>"<th>" + esc(r.name) + "</th>").join("") + "</tr></thead><tbody>";
        cmp.rows.forEach(x=>{
          const v0 = x.vals[0];
          setupCmpHtml += "<tr><td>" + esc(x.label) + "</td><td>" + esc(x.unit) + "</td>";
          x.vals.forEach((v,i)=>{
            if(v == null){ setupCmpHtml += "<td>—</td>"; return; }
            let cell = num(v,2);
            if(i > 0 && v0 != null){
              const dv = v - v0;
              cell += " <span class='mut'>(" + (dv>=0?"+":"") + num(dv,2) + ")</span>";
            }
            setupCmpHtml += "<td>" + cell + "</td>";
          });
          setupCmpHtml += "</tr>";
        });
        setupCmpHtml += "</tbody></table>";
      }
    }
  }catch(e){ /* 方案库不可用时保持占位文案，绝不让报告导出失败 — if the setup library is unavailable, keep the placeholder text; the report export must never fail */ }

  /* 基准差异（若已锁定基准线） — baseline delta (if a baseline is locked) */
  let baseHtml = "";
  if(S.hasBaseline){
    const items = [["Camber gain CG","cg","°/25mm"],["Bump steer BS","bs","°/25mm"],["Motion ratio MR","mr","-"],
                   ["Roll center RC","rcH","mm"],["Wheel rate WR","kw","N/mm"]];
    baseHtml = "<h2>Baseline delta</h2><table><thead><tr><th>Item</th><th>Current (ACTIVE)</th><th>Baseline (BASE)</th><th>Delta (DELTA)</th></tr></thead><tbody>";
    items.forEach(([lb,k,u])=>{
      const g = id => { const e = document.getElementById(id); return e ? e.textContent : "—"; };
      baseHtml += "<tr><td>" + lb + " <span class='mut'>" + u + "</span></td><td>" + esc(g("df_a_"+k))
        + "</td><td>" + esc(g("df_b_"+k)) + "</td><td class='hot'>" + esc(g("df_d_"+k)) + "</td></tr>";
    });
    baseHtml += "</tbody></table>";
  }

  /* 评分与建议 — score & advice */
  let scoreHtml = "<p class='mut'>Evaluation data unavailable (missing geometry or sweep results).</p>";
  if(data && data.dimScores){
    scoreHtml = "<table><thead><tr><th>Dimension</th><th>Score</th></tr></thead><tbody>";
    for(const k in data.dimScores) scoreHtml += "<tr><td>" + esc(k) + "</td><td>" + Math.round(data.dimScores[k]) + " / 100</td></tr>";
    scoreHtml += "<tr class='tot'><td>Overall score</td><td>" + (data.overall != null ? data.overall : "—") + " / 100</td></tr></tbody></table>";
  }

  const engineTxt = (typeof ENG !== "undefined" && ENG.ok)
    ? ("Engine connected: " + esc(ENG.url)) : "Built-in JS solver (offline)";
  const stamp = new Date().toLocaleString("zh-CN");
  /* 残差取真源：状态栏同源求解残差（07-plots 写 sbRes），缺省回退到当前轴 metrics — residual from the true source: the same solve residual as the status bar (07-plots writes sbRes), falling back to the current axis metrics when absent */
  const resEl = document.getElementById("sbRes");
  const residual = (resEl && resEl.textContent && resEl.textContent !== "—")
    ? resEl.textContent
    : ((SIM.mFR && typeof SIM.mFR.res === "number") ? num(SIM.mFR.res, 5) : "—");

  const rowsTbl = (rows) => "<table><tbody>" + rows.map(r =>
    "<tr><th>" + esc(r[0]) + "</th><td>" + esc(r[1]) + "</td></tr>").join("") + "</tbody></table>";

  const html = `<!DOCTYPE html><html lang="zh-CN"><head><meta charset="utf-8">
<title>LABSUS Design Report · ${esc(S.vehicleType)} · ${esc(stamp)}</title>
<style>
  :root{color-scheme:light}
  body{font:13px/1.6 -apple-system,"Segoe UI",system-ui,"Microsoft YaHei",sans-serif;color:#1a1f28;background:#fff;margin:0;padding:28px 34px;max-width:1000px}
  h1{font-size:20px;margin:0 0 2px} h2{font-size:14px;margin:22px 0 6px;padding-bottom:3px;border-bottom:1px solid #d8dee8}
  .meta{color:#5b6675;font-size:12px;margin-bottom:14px}
  table{border-collapse:collapse;width:100%;margin:4px 0 10px;font-size:12px}
  th,td{border:1px solid #e2e8f0;padding:4px 8px;text-align:left;vertical-align:top}
  thead th{background:#f4f7fb;font-weight:600}
  tbody th{background:#fafcfe;font-weight:500;width:32%;white-space:nowrap}
  td.hot{color:#c2410c;font-weight:600} tr.tot td{font-weight:700;background:#f4f7fb}
  .mut{color:#64748b;font-size:11.5px}
  .grid{display:flex;flex-wrap:wrap;gap:10px}
  .grid figure{margin:0;border:1px solid #e2e8f0;border-radius:6px;padding:6px;background:#fbfdff}
  .grid img{display:block;max-width:460px;height:auto}
  figcaption{font-size:11px;color:#5b6675;margin-top:4px}
  .note{margin-top:22px;padding:10px 12px;border-left:3px solid #94a3b8;background:#f8fafc;font-size:11.5px;color:#475569}
  @media print{body{padding:0;max-width:none} h2{page-break-after:avoid} figure,table{page-break-inside:avoid}}
</style></head><body>
<h1>LABSUS Suspension Design Report</h1>
<div class="meta">Vehicle: <b>${esc(vehName)}</b> · Generated: ${esc(stamp)} · Solver: ${engineTxt} · Max constraint residual: ${esc(residual)} mm</div>

<h2>1. Vehicle & tuning parameters</h2>
${rowsTbl(vehRows)}

<h2>2. Overall score</h2>
${scoreHtml}

<h2>3. Key design KPIs</h2>
${rowsTbl(kpiRows)}

<h2>4. Quasi-static handling</h2>
${rowsTbl(qsRows)}

<h2>5. View snapshots & characteristic curves</h2>
<div class="grid">${imgs.map(x => `<figure><img src="${x.url}" alt="${esc(x.label)}"><figcaption>${esc(x.label)}</figcaption></figure>`).join("") || "<p class='mut'>No canvases available.</p>"}</div>

<h2>6. Sensitivity sweep</h2>
${sensHtml}

<h2>7. Two-parameter interaction sweep</h2>
${inter2Html}

<h2>8. Multi-setup comparison</h2>
${setupCmpHtml}
${baseHtml}

<div class="note">
<b>Model boundaries & disclaimers</b> (avoid conclusions beyond the model):<br>
① Steady-state handling quantities are <b>linear-region quasi-static solutions</b>: slip angle α is linearly converted from axle cornering stiffness and excludes tire nonlinear saturation, transient relaxation and thermal effects;<br>
② Load sensitivity is modeled as <b>LS dropping per doubling of μ</b>; the reference load is the vehicle mean static wheel load mT·g/4;<br>
③ Not included: bushing rotational compliance (δ_rot≡0), camber→grip coupling, road-spectrum excitation, refined aerodynamic ground effect;<br>
④ Sensitivity sweeps are <b>single-parameter local sweeps</b> (all other parameters frozen) with no parameter interactions; cross-parameter conclusions require multivariate validation;<br>
⑤ The dual kernels (Python engine / front-end JS) are cross-checked via <code>tphys_parity</code>; worst relative difference ≈ 0.5%.
</div>
</body></html>`;

  /* 打开新窗口；被拦截则退回 Blob 下载 — open a new window; fall back to a Blob download if blocked */
  let w = null;
  try{ w = window.open("", "_blank"); }catch(e){ w = null; }
  if(w && w.document){
    w.document.open(); w.document.write(html); w.document.close();
    try{ w.focus(); }catch(e){}
  } else {
    try{
      const blob = new Blob([html], {type:"text/html;charset=utf-8"});
      const a = document.createElement("a");
      a.href = URL.createObjectURL(blob);
      a.download = "LABSUS_Design_Report_" + S.vehicleType + "_" + new Date().toISOString().slice(0,10) + ".html";
      document.body.appendChild(a); a.click(); document.body.removeChild(a);
      setTimeout(()=>URL.revokeObjectURL(a.href), 4000);
    }catch(e){
      const st = document.getElementById("evalOverallScore");
      if(st) st.textContent = "Export failed: " + e.message;
    }
  }
  /* 返回完整 HTML 串（便于自动化取证：把返回值写进隐藏节点再 --dump-dom 读回来），
     而不是长度——旧实现 return html.length，导致"看起来成功、实际拿到的是个数字"。
  — Return the full HTML string (easy for automated forensics: write the return value into a hidden node and read it back with --dump-dom) rather than a length — the old return html.length made "success" while returning a number. */
  return html;
}
window.exportDesignReport = exportDesignReport;

function drawRadarChart(scores){
  const cv = document.getElementById("evalRadarCanvas");
  if(!cv) return;
  const ctx = cv.getContext("2d");
  const w = cv.width, h = cv.height;
  ctx.clearRect(0, 0, w, h);

  const cx = w / 2, cy = h / 2, R = w * 0.38;
  const keys = Object.keys(scores);
  const total = keys.length;
  const angleStep = (Math.PI * 2) / total;

  ctx.strokeStyle = "rgba(255, 255, 255, 0.1)";
  ctx.lineWidth = 1;
  for(let layer = 1; layer <= 4; layer++){
    ctx.beginPath();
    const r = (R / 4) * layer;
    for(let i = 0; i < total; i++){
      const ang = i * angleStep - Math.PI / 2;
      const x = cx + r * Math.cos(ang), y = cy + r * Math.sin(ang);
      if(i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.closePath();
    ctx.stroke();
  }

  for(let i = 0; i < total; i++){
    const ang = i * angleStep - Math.PI / 2;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + R * Math.cos(ang), cy + R * Math.sin(ang));
    ctx.stroke();

    const labelX = cx + (R + 22) * Math.cos(ang);
    const labelY = cy + (R + 22) * Math.sin(ang);
    ctx.fillStyle = "#8b949e";
    ctx.font = "10.5px ui-monospace, sans-serif";
    ctx.textAlign = Math.abs(Math.cos(ang)) < 0.2 ? "center" : (Math.cos(ang) > 0 ? "left" : "right");
    ctx.textBaseline = "middle";
    ctx.fillText(keys[i].split(" ")[0], labelX, labelY);
  }

  ctx.beginPath();
  for(let i = 0; i < total; i++){
    const ang = i * angleStep - Math.PI / 2;
    const scorePct = Math.min(100, Math.max(10, scores[keys[i]])) / 100;
    const r = R * scorePct;
    const x = cx + r * Math.cos(ang), y = cy + r * Math.sin(ang);
    if(i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  }
  ctx.closePath();
  ctx.fillStyle = "rgba(88, 166, 255, 0.25)";
  ctx.fill();
  ctx.strokeStyle = "#58a6ff";
  ctx.lineWidth = 2.2;
  ctx.stroke();

  for(let i = 0; i < total; i++){
    const ang = i * angleStep - Math.PI / 2;
    const scorePct = Math.min(100, Math.max(10, scores[keys[i]])) / 100;
    const r = R * scorePct;
    const x = cx + r * Math.cos(ang), y = cy + r * Math.sin(ang);
    ctx.beginPath();
    ctx.arc(x, y, 4, 0, Math.PI * 2);
    ctx.fillStyle = "#58a6ff";
    ctx.fill();
    ctx.strokeStyle = "#fff";
    ctx.lineWidth = 1.5;
    ctx.stroke();
  }
}

/* =====================================================================
   LABSUS 15-DOF HARDCORE MULTI-BODY VEHICLE DYNAMICS ENGINE
   ===================================================================== */
/* =====================================================================
   LABSUS HARDCORE 15-DOF MULTI-BODY VEHICLE DYNAMICS ENGINE
   Coordinate Frame: +Y Forward, +X Right, +Z Up (Pitch around X, Roll around Y, Yaw around Z)
   ===================================================================== */
/* =====================================================================
   LABSUS HARDCORE 15-DOF MULTI-BODY VEHICLE DYNAMICS ENGINE
   Coordinate Frame: +Y Forward, +X Right, +Z Up (Pitch around X, Roll around Y, Yaw around Z)
   ===================================================================== */

/* =====================================================================
   UNIVERSAL AUTONOMOUS PILOT (ADAS / AD)
   ===================================================================== */
class TrackPath {
  getLookahead(x, y, v) { return null; }
}

class CirclePath extends TrackPath {
  constructor(radius, dir, speedMs) {
    super();
    this.R = radius;
    this.dir = dir; // -1: CCW, 1: CW
    this.targetSpeed = speedMs;
  }
  
  getLookahead(x, y, v) {
    const r_cur = Math.hypot(x, y);
    const theta = (r_cur < 1e-3) ? 0 : Math.atan2(y, x);
    
    const psi_tangent = (this.dir === -1) ? theta : (theta + Math.PI);
    
    const tx = -Math.sin(psi_tangent);
    const ty = Math.cos(psi_tangent);
    const nx_norm = ty;
    const ny_norm = -tx;
    
    const nx = this.R * Math.cos(theta);
    const ny = this.R * Math.sin(theta);
    
    const e_y = (x - nx) * nx_norm + (y - ny) * ny_norm;
    
    return {
      targetHeading: psi_tangent,
      targetCurvature: (this.dir === -1) ? (-1.0 / this.R) : (1.0 / this.R), 
      targetSpeed: this.targetSpeed,
      crossTrackError: e_y
    };
  }
}

class StraightPath extends TrackPath {
  constructor(speedMs = 23.6, scenario = "comprehensive", params = {}) {
    super();
    this.targetSpeed = speedMs;
    this.scenario = scenario || "comprehensive";
    this.params = Object.assign({
      repeatCount: 10,
      rampHeight: 1.10,
      rampLen: 14.0,
      rampSpacing: 65.0,
      bumpSubType: "staggered",
      bumpHeight: 0.060,
      bumpSpacing: 4.0,
      mooseLaneWidth: 2.8,
      mooseOffset: 3.5,
      mooseSpeedKmh: 75.0,
      washboardWavelength: 1.25,
      washboardAmplitude: 0.035,
      potholeDepth: -0.055,
      undulatingWavelength: 9.0,
      undulatingAmplitude: 0.035,
      undulatingCrossAmp: 0.035,
      undulatingPhase: 180,
      undulatingType: "staggered_moguls",
      muLeft: 1.35,
      muRight: 0.28,
      grade: 0.18,
      customSegments: null
    }, params);

    this.segments = [];
    this.ramps = [];
    this.bumps = [];
    this.mooseSections = [];
    this.washboardSections = [];
    this.undulatingSections = [];
    this.splitMuSections = [];
    this.slopeSections = [];
    this.gantries = [];
    this._cones = [];
    this._coneHits = new Set();

    this.buildTrack();
  }

  setScenario(scenario, params = {}) {
    this.scenario = scenario;
    if (params) Object.assign(this.params, params);
    this._coneHits.clear();
    this.buildTrack();
  }

  buildTrack() {
    this.segments = [];
    this.ramps = [];
    this.bumps = [];
    this.mooseSections = [];
    this.washboardSections = [];
    this.undulatingSections = []; // G28-fix：必须随场景重建清空，否则波浪永久叠加到其他所有场景 — must be cleared on every scene rebuild, otherwise the waves permanently stack onto all other scenarios
    this.splitMuSections = [];
    this.slopeSections = [];
    this.gantries = [];
    this._cones = [];

    const sc = this.scenario;
    const rCount = Math.max(1, Math.min(50, this.params.repeatCount || 10));

    if (sc === "comprehensive") {
      // 🏆 默认：全工况综合连环大试验场 (7 大工况连续串联) — default: full-spectrum combined proving ground (7 load cases chained in series)
      let curY = 0;
      
      // 1. 弹射加速与制动区 (0-100 & 100-0) — launch-acceleration & braking zone
      this.segments.push({
        id: "seg_accel", type: "accel_brake", name: "⚡ Launch acceleration & braking",
        yStart: curY, yEnd: curY + 110.0, targetSpeed: 38.0, triggerY: curY + 75.0
      });
      this.gantries.push({ y: curY, text: "🏁 START · 0-100 ACCEL & BRAKE" });
      curY += 120.0;

      // 2. 三连飞坡跳台 (3x Jump Ramps)
      this.gantries.push({ y: curY, text: "🚀 TRIPLE JUMP RAMPS · 3 consecutive jump ramps" });
      for (let i = 0; i < 3; i++) {
        const ry = curY + 15.0 + i * 65.0;
        const rH = this.params.rampHeight || 1.10;
        this.ramps.push({
          id: `comp_ramp_${i}`, y0: ry, len: 14.0, h: rH, landingY: ry + 28.0, landingLen: 16.0, index: i + 1, total: 3
        });
      }
      this.segments.push({
        id: "seg_ramp", type: "jump_ramp", name: "🚀 Triple jump ramps (3x)",
        yStart: curY, yEnd: curY + 220.0, targetSpeed: this.targetSpeed
      });
      curY += 230.0;

      // 3. 连续交错坎与减速丘 (Speed Bumps & Cleats)
      this.gantries.push({ y: curY, text: "🚧 SPEED BUMPS & CLEATS · consecutive bumps & cleats" });
      const bH = this.params.bumpHeight || 0.060;
      for (let i = 0; i < 6; i++) {
        const by = curY + 15.0 + i * 8.0;
        this.bumps.push({
          y0: by, side: (i % 2 === 0 ? "L" : "R"), h: bH,
          xMin: (i % 2 === 0 ? -2.5 : 0.1), xMax: (i % 2 === 0 ? -0.1 : 2.5)
        });
      }
      this.segments.push({
        id: "seg_bumps", type: "bumps_cleats", name: "🚧 Consecutive staggered speed bumps (6x)",
        yStart: curY, yEnd: curY + 80.0, targetSpeed: 16.7
      });
      curY += 90.0;

      // 4. 连续双移线麋鹿避障 (2x Moose Tests, alternating left and right)
      this.gantries.push({ y: curY, text: "🦌 ISO 3888-2 MOOSE TEST · consecutive moose avoidance" });
      for (let i = 0; i < 2; i++) {
        const my = curY + 15.0 + i * 70.0;
        const dir = (i % 2 === 0 ? -1 : 1);
        this.mooseSections.push({
          id: `comp_moose_${i}`, y0: my, offset: dir * (this.params.mooseOffset || 3.5),
          width: this.params.mooseLaneWidth || 2.8, len: 60.0, index: i + 1, total: 2
        });
      }
      this.segments.push({
        id: "seg_moose", type: "moose_test", name: "🦌 Consecutive double lane-change (2x Moose)",
        yStart: curY, yEnd: curY + 160.0, targetSpeed: (this.params.mooseSpeedKmh || 75.0) / 3.6
      });
      curY += 170.0;

      // 5. 搓板与暗坑耐久路 (Washboard & Potholes)
      this.gantries.push({ y: curY, text: "🪨 WASHBOARD & POTHOLES · washboard & pothole endurance" });
      this.washboardSections.push({
        id: "comp_wash", y0: curY + 10.0, len: 45.0,
        wavelength: this.params.washboardWavelength || 1.25,
        amp: this.params.washboardAmplitude || 0.035,
        potholeDepth: this.params.potholeDepth || -0.055
      });
      this.segments.push({
        id: "seg_wash", type: "washboard_potholes", name: "🪨 Washboard corrugations & hidden potholes",
        yStart: curY, yEnd: curY + 80.0, targetSpeed: 13.9
      });
      curY += 90.0;

      // 6. 对开冰雪沥青路 ($\mu$-Split Surface)
      this.gantries.push({ y: curY, text: "❄️ SPLIT-MU SURFACE · split-μ ice/asphalt braking" });
      this.splitMuSections.push({
        id: "comp_split", y0: curY + 10.0, len: 90.0,
        muLeft: this.params.muLeft || 1.35, muRight: this.params.muRight || 0.28
      });
      this.segments.push({
        id: "seg_split", type: "split_mu", name: "❄️ Split-μ surface grip imbalance",
        yStart: curY, yEnd: curY + 100.0, targetSpeed: 19.4
      });
      curY += 110.0;

      // 7. 连续极限坡度爬坡 (Slope Climb)
      this.gantries.push({ y: curY, text: "⛰️ EXTREME SLOPE CLIMB · extreme steep-slope climb" });
      this.slopeSections.push({
        id: "comp_slope", y0: curY + 10.0, len: 120.0, grade: this.params.grade || 0.20
      });
      this.segments.push({
        id: "seg_slope", type: "slope_climb", name: "⛰️ Consecutive extreme steep-slope climb",
        yStart: curY, yEnd: curY + 130.0, targetSpeed: 18.0
      });
      curY += 140.0;

      this.gantries.push({ y: curY, text: "🏁 FINISH · LAP COMPLETE" });
      this.totalLength = curY;

    } else if (sc === "jump_ramp") {
      // 🚀 连续飞坡极限界 (连续 10 个飞坡跳台) — consecutive jump-ramp gauntlet (10 jumps in a row)
      const rH = this.params.rampHeight || 1.10;
      const rLen = this.params.rampLen || 14.0;
      const rY0 = this.params.rampY !== undefined ? this.params.rampY : 20.0;
      const sp = this.params.rampSpacing || 65.0;
      let curY = rY0;
      this.gantries.push({ y: 0, text: `🚀 ${rCount}x CONSECUTIVE JUMP RAMPS · 10-jump-ramp gauntlet` });

      for (let i = 0; i < rCount; i++) {
        const ry = curY;
        this.ramps.push({
          id: `ramp_${i}`, y0: ry, len: rLen, h: rH, landingY: ry + rLen + 14.0, landingLen: 16.0, index: i + 1, total: rCount
        });
        this.segments.push({
          id: `seg_ramp_${i}`, type: "jump_ramp", name: `🚀 Jump ramp ${i + 1}/${rCount}`,
          yStart: ry - 10.0, yEnd: ry + sp - 10.0, targetSpeed: this.targetSpeed
        });
        curY += sp;
      }
      this.totalLength = curY + 50.0;

    } else if (sc === "moose_test") {
      // 🦌 连续麋鹿避障测试 (连续 5~10 组左右交替双移线) — consecutive moose tests (5-10 alternating left/right double lane-changes)
      const off = Math.abs(this.params.mooseOffset || 3.5);
      const w = this.params.mooseLaneWidth || 2.8;
      const mY0 = this.params.mooseStartY !== undefined ? this.params.mooseStartY : 20.0;
      let curY = mY0;
      this.gantries.push({ y: 0, text: `🦌 ${rCount}x CONSECUTIVE MOOSE TESTS · consecutive moose avoidance` });

      for (let i = 0; i < rCount; i++) {
        const my = curY;
        const dir = (i % 2 === 0 ? -1 : 1);
        this.mooseSections.push({
          id: `moose_${i}`, y0: my, offset: dir * off, width: w, len: 60.0, index: i + 1, total: rCount
        });
        this.segments.push({
          id: `seg_moose_${i}`, type: "moose_test", name: `🦌 Moose double lane-change ${i + 1}/${rCount} (${dir < 0 ? 'left' : 'right'})`,
          yStart: my - 10.0, yEnd: my + 68.0, targetSpeed: (this.params.mooseSpeedKmh || 75.0) / 3.6
        });
        curY += 75.0;
      }
      this.totalLength = curY + 50.0;

    } else if (sc === "bumps_cleats") {
      // 🚧 连续过坎与减速带 — consecutive bumps & speed humps
      const bH = this.params.bumpHeight || 0.060;
      const bY0 = this.params.bumpStartY !== undefined ? this.params.bumpStartY : 20.0;
      const bSp = this.params.bumpSpacing || 4.0;
      let curY = bY0;
      this.gantries.push({ y: 0, text: `🚧 ${rCount * 4}x SPEED BUMPS & CLEATS · consecutive bump test` });

      for (let i = 0; i < rCount * 4; i++) {
        const by = curY + i * bSp;
        this.bumps.push({
          y0: by, side: (i % 2 === 0 ? "L" : "R"), h: bH,
          xMin: (i % 2 === 0 ? -2.5 : 0.1), xMax: (i % 2 === 0 ? -0.1 : 2.5)
        });
      }
      this.segments.push({
        id: "seg_bumps_all", type: "bumps_cleats", name: `🚧 Consecutive staggered bumps (${rCount * 4} in a row)`,
        yStart: 0, yEnd: curY + rCount * 4 * bSp, targetSpeed: this.targetSpeed
      });
      this.totalLength = curY + rCount * 4 * bSp + 50.0;

    } else if (sc === "washboard_potholes") {
      // 🪨 连续搓板与深坑路 — continuous washboard & deep-pothole road
      const wY0 = this.params.washboardStartY !== undefined ? this.params.washboardStartY : 20.0;
      let curY = wY0;
      this.gantries.push({ y: 0, text: "🪨 CONTINUOUS WASHBOARD & POTHOLES · continuous washboard & potholes" });
      const wLen = 120.0;
      this.washboardSections.push({
        id: "wash_main", y0: curY, len: wLen,
        wavelength: this.params.washboardWavelength || 1.25,
        amp: this.params.washboardAmplitude || 0.035,
        potholeDepth: this.params.potholeDepth || -0.055
      });
      this.segments.push({
        id: "seg_wash_all", type: "washboard_potholes", name: "🪨 Continuous washboard & pothole endurance road",
        yStart: 0, yEnd: curY + wLen + 50.0, targetSpeed: this.targetSpeed
      });
      this.totalLength = curY + wLen + 80.0;

    } else if (sc === "accel_brake") {
      this.gantries.push({ y: 0, text: "⚡ 0-100 ACCELERATION & MAXIMUM BRAKING" });
      this.segments.push({
        id: "seg_accel", type: "accel_brake", name: "⚡ 0-100 launch accel & full braking",
        yStart: 0, yEnd: 150.0, targetSpeed: 45.0, triggerY: this.params.accelBrakeTriggerY || 90.0
      });
      this.totalLength = 200.0;

    } else if (sc === "split_mu") {
      this.gantries.push({ y: 0, text: "❄️ SPLIT-MU RUNWAY · split-μ traction stability" });
      this.splitMuSections.push({
        id: "split_main", y0: 10.0, len: 180.0,
        muLeft: this.params.muLeft || 1.35, muRight: this.params.muRight || 0.28
      });
      this.segments.push({
        id: "seg_split", type: "split_mu", name: "❄️ Split-μ surface grip imbalance test",
        yStart: 0, yEnd: 200.0, targetSpeed: this.targetSpeed
      });
      this.totalLength = 240.0;

    } else if (sc === "slope_climb") {
      this.gantries.push({ y: 0, text: "⛰️ CONTINUOUS SLOPE CLIMB · continuous steep-slope climb" });
      this.slopeSections.push({
        id: "slope_main", y0: 10.0, len: 300.0, grade: this.params.grade || 0.18
      });
      this.segments.push({
        id: "seg_slope", type: "slope_climb", name: "⛰️ Consecutive extreme slope climb",
        yStart: 0, yEnd: 320.0, targetSpeed: this.targetSpeed
      });
      this.totalLength = 350.0;

    } else if (sc === "undulating_road") {
      // 🌊 越野连续交错波浪起伏荒原 (Off-Road Staggered Moguls & Articulation)
      // G28-fix 参数重标定：波浪必须满足 a = A·(2πv/λ)² < g 才不会把车抛飞；
      // 默认 0.035m/9m @45km/h ≈ 2.7m/s²（舒缓旷野起伏），旧 0.14m/4.8m ≈ 2.4g 必飞车
      // — G28-fix recalibration: the wave must satisfy a = A(2πv/λ)² < g or the car gets launched; default 0.035m/9m @45km/h ≈ 2.7m/s² (gentle open-terrain undulation), the old 0.14m/4.8m ≈ 2.4g always launched the car
      const wL = this.params.undulatingWavelength || 9.0;
      const amp = this.params.undulatingAmplitude !== undefined ? this.params.undulatingAmplitude : 0.035;
      const crossAmp = this.params.undulatingCrossAmp !== undefined ? this.params.undulatingCrossAmp : 0.035;
      const phaseDeg = this.params.undulatingPhase !== undefined ? this.params.undulatingPhase : 180;
      const uType = this.params.undulatingType || "staggered_moguls";
      const uY0 = 3.0;
      const totalLen = Math.max(250.0, rCount * wL * 2.0);

      this.undulatingSections.push({
        id: "undul_main", y0: uY0, len: totalLen,
        wavelength: wL, amp: amp, crossAmp: crossAmp, phaseDeg: phaseDeg, uType: uType,
        repeatCount: rCount
      });

      this.gantries.push({ y: 0, text: `🌊 OFF-ROAD MOGULS · off-road staggered wave packs (wavelength ${wL.toFixed(1)}m · wave height ${(amp*1000).toFixed(0)}mm)` });

      for (let i = 0; i < rCount; i++) {
        const segY = uY0 + i * wL;
        this.segments.push({
          id: `seg_undul_${i}`, type: "undulating_road",
          name: `🌊 Off-road staggered wave packs (${i + 1}/${rCount}) · left/right anti-phase articulation · wave height ${(amp*1000).toFixed(0)}mm`,
          yStart: segY, yEnd: segY + wL, targetSpeed: this.targetSpeed
        });
      }
      this.totalLength = uY0 + totalLen + 50.0;
    }

    this.rebuildCones();
  }

  rebuildCones() {
    this._cones = [];
    if (!this.mooseSections.length) return;

    for (const sec of this.mooseSections) {
      const sy = sec.y0;
      const w = sec.width || 2.8;
      const off = sec.offset || -3.5;
      const prefix = sec.id;

      // Section 1: Entry Lane (Length 12m)
      for (let y = sy; y <= sy + 12; y += 3.0) {
        this._cones.push({ id: `${prefix}_eL_${y}`, x: -w / 2, y: y, side: "L" });
        this._cones.push({ id: `${prefix}_eR_${y}`, x: w / 2, y: y, side: "R" });
      }
      // Section 3: Offset Evasion Lane (Length 11m, lateral offset)
      const s3_start = sy + 12 + 13.5;
      const s3_end = s3_start + 11.0;
      for (let y = s3_start; y <= s3_end; y += 2.75) {
        this._cones.push({ id: `${prefix}_mL_${y}`, x: off - (w * 1.05) / 2, y: y, side: "L" });
        this._cones.push({ id: `${prefix}_mR_${y}`, x: off + (w * 1.05) / 2, y: y, side: "R" });
      }
      // Section 5: Return / Exit Lane (Length 12m)
      const s5_start = s3_end + 12.5;
      const s5_end = s5_start + 12.0;
      for (let y = s5_start; y <= s5_end; y += 3.0) {
        this._cones.push({ id: `${prefix}_xL_${y}`, x: -(w * 1.15) / 2, y: y, side: "L" });
        this._cones.push({ id: `${prefix}_xR_${y}`, x: (w * 1.15) / 2, y: y, side: "R" });
      }
    }
  }

  checkConeCollisions(posX, posY, psi, vehW = 1.8, vehL = 3.8) {
    if (!this._cones.length) return 0;
    const cPsi = Math.cos(psi), sPsi = Math.sin(psi);
    const halfW = vehW / 2 + 0.15;
    const halfL = vehL / 2 + 0.15;

    for (const c of this._cones) {
      if (this._coneHits.has(c.id)) continue;
      // Quick radius reject
      if (Math.abs(c.y - posY) > 4.5 || Math.abs(c.x - posX) > 3.5) continue;
      const dx = c.x - posX;
      const dy = c.y - posY;
      const lx = dx * cPsi + dy * sPsi;
      const ly = -dx * sPsi + dy * cPsi;
      if (Math.abs(lx) <= halfW && Math.abs(ly) <= halfL) {
        this._coneHits.add(c.id);
      }
    }
    return this._coneHits.size;
  }

  getActiveSegment(y) {
    if (!this.segments.length) return { name: "Straight road", type: "straight", index: 1, total: 1, progress: 0 };
    for (let i = 0; i < this.segments.length; i++) {
      const seg = this.segments[i];
      if (y >= seg.yStart && y <= seg.yEnd) {
        const prog = (y - seg.yStart) / Math.max(1, seg.yEnd - seg.yStart);
        return { name: seg.name, type: seg.type, index: i + 1, total: this.segments.length, progress: prog, seg };
      }
    }
    const last = this.segments[this.segments.length - 1];
    if (y > last.yEnd) {
      return { name: "🏁 Finish straight", type: "finish", index: this.segments.length, total: this.segments.length, progress: 1.0 };
    }
    return { name: this.segments[0].name, type: this.segments[0].type, index: 1, total: this.segments.length, progress: 0 };
  }

  getLookahead(x, y, v) {
    // 1. Check if inside any Moose Section
    for (const sec of this.mooseSections) {
      const sy = sec.y0;
      const off = sec.offset;
      const s1_end = sy + 12.0;
      const s2_end = s1_end + 13.5;
      const s3_end = s2_end + 11.0;
      const s4_end = s3_end + 12.5;

      if (y >= sy && y <= s4_end + 10.0) {
        let targetX = 0;
        let targetHeading = 0;
        let targetCurvature = 0;

        if (y <= s1_end) {
          targetX = 0; targetHeading = 0; targetCurvature = 0;
        } else if (y <= s2_end) {
          const t = (y - s1_end) / 13.5;
          const s_curve = 0.5 - 0.5 * Math.cos(Math.PI * t);
          const ds_dy = (0.5 * Math.PI / 13.5) * Math.sin(Math.PI * t);
          const d2s_dy2 = (0.5 * Math.PI * Math.PI / (13.5 * 13.5)) * Math.cos(Math.PI * t);
          targetX = off * s_curve;
          targetHeading = Math.atan2(off * ds_dy, 1.0);
          targetCurvature = (off * d2s_dy2) / Math.pow(1.0 + (off * ds_dy) ** 2, 1.5);
        } else if (y <= s3_end) {
          targetX = off; targetHeading = 0; targetCurvature = 0;
        } else if (y <= s4_end) {
          const t = (y - s3_end) / 12.5;
          const s_curve = 0.5 + 0.5 * Math.cos(Math.PI * t);
          const ds_dy = -(0.5 * Math.PI / 12.5) * Math.sin(Math.PI * t);
          const d2s_dy2 = -(0.5 * Math.PI * Math.PI / (12.5 * 12.5)) * Math.cos(Math.PI * t);
          targetX = off * s_curve;
          targetHeading = Math.atan2(off * ds_dy, 1.0);
          targetCurvature = (off * d2s_dy2) / Math.pow(1.0 + (off * ds_dy) ** 2, 1.5);
        } else {
          targetX = 0; targetHeading = 0; targetCurvature = 0;
        }

        const e_y = (x - targetX);
        const targetSpeed = (this.params.mooseSpeedKmh || 75.0) * (1000 / 3600);
        return { targetHeading, targetCurvature, targetSpeed, crossTrackError: e_y, targetX };
      }
    }

    // 2. Check if inside Accel & Brake Segment
    for (const seg of this.segments) {
      if (seg.type === "accel_brake" && y >= seg.yStart && y <= seg.yEnd) {
        const triggerY = seg.triggerY || (seg.yStart + 75.0);
        const targetSpeed = (y < triggerY) ? seg.targetSpeed : 0.0;
        return { targetHeading: 0, targetCurvature: 0, targetSpeed, crossTrackError: x, targetX: 0 };
      }
    }

    // Default straight path
    return { targetHeading: 0, targetCurvature: 0, targetSpeed: this.targetSpeed, crossTrackError: x, targetX: 0 };
  }

  getRoadElevation(x, y) {
    let z_road = 0.0;
    let isKerb = false;
    let isBump = false;
    let isPothole = false;
    let isRamp = false;
    let mu = 1.35;

    // 1. Jump Ramps
    for (const r of this.ramps) {
      if (y >= r.y0 && y <= (r.y0 + r.len) && Math.abs(x) <= 3.8) {
        isRamp = true;
        const t = (y - r.y0) / r.len;
        z_road = Math.max(z_road, r.h * (t * t * (3.0 - 2.0 * t)));
      } else if (y > (r.y0 + r.len) && y < r.landingY) {
        // Drop-off flight gap: ground elevation is 0
      } else if (y >= r.landingY && y <= (r.landingY + r.landingLen) && Math.abs(x) <= 4.0) {
        const t = (y - r.landingY) / r.landingLen;
        z_road = Math.max(z_road, (1.0 - t) * 0.04);
      }
    }

    // 2. Speed Bumps & Cleats
    for (const bp of this.bumps) {
      if (y >= bp.y0 && y <= bp.y0 + 1.0 && x >= bp.xMin && x <= bp.xMax) {
        isBump = true;
        const t = (y - bp.y0) / 1.0;
        z_road = Math.max(z_road, bp.h * Math.sin(Math.PI * t) ** 2);
      }
    }

    // 3. Washboard & Potholes
    for (const w of this.washboardSections) {
      if (y >= w.y0 && y <= (w.y0 + w.len) && Math.abs(x) <= 3.8) {
        isBump = true;
        z_road += w.amp * Math.sin((2.0 * Math.PI * (y - w.y0)) / w.wavelength);
      } else if (y > (w.y0 + w.len) && y <= (w.y0 + w.len + 50.0)) {
        const pits = [
          { y0: w.y0 + w.len + 8.0, x0: -0.75, rx: 0.6, ry: 0.8 },
          { y0: w.y0 + w.len + 18.0, x0: 0.85, rx: 0.65, ry: 0.85 },
          { y0: w.y0 + w.len + 28.0, x0: -0.45, rx: 0.7, ry: 0.9 },
          { y0: w.y0 + w.len + 38.0, x0: 0.60, rx: 0.6, ry: 0.8 }
        ];
        for (const p of pits) {
          const dx = (x - p.x0) / p.rx;
          const dy = (y - p.y0) / p.ry;
          const r2 = dx * dx + dy * dy;
          if (r2 <= 1.0) {
            isPothole = true;
            const factor = Math.cos((Math.PI / 2) * Math.sqrt(r2));
            z_road = Math.min(z_road, w.potholeDepth * factor);
          }
        }
        if (Math.abs(x) <= 3.8) {
          z_road += 0.005 * Math.sin(y * 11.3) * Math.cos(x * 9.7);
        }
      }
    }

    // 4. Split-Mu Friction
    for (const sp of this.splitMuSections) {
      if (y >= sp.y0 && y <= (sp.y0 + sp.len)) {
        mu = (x > 0 ? sp.muRight : sp.muLeft);
      }
    }

    // 5. Undulating Road Surface (自然非铺装起伏荒原 · 左右连续交错波浪包) — natural unpaved undulating terrain · left/right staggered wave packs
    if (this.undulatingSections && this.undulatingSections.length) {
      for (const u of this.undulatingSections) {
        if (y >= u.y0 && y <= (u.y0 + u.len)) {
          isBump = false; // 自然地表起伏，非离散碰撞坎 — natural terrain undulation, not a discrete cleat
          const dy = y - u.y0;
          const k_y = (2.0 * Math.PI) / Math.max(1.5, u.wavelength);
          const phaseRad = (u.phaseDeg !== undefined ? u.phaseDeg : 180) * (Math.PI / 180);

          // 左右反相交错起伏波浪 (Staggered Moguls)
          // 左轮 (x < 0) 遇波峰时，右轮 (x > 0) 正好进入波谷，激发悬架极限交叉轴运动 — when the left wheel (x < 0) is on a crest the right wheel (x > 0) is exactly in a trough, exciting extreme cross-axle suspension motion
          const leftWave = u.amp * Math.sin(k_y * dy);
          const rightWave = u.amp * Math.sin(k_y * dy + phaseRad);

          // 横向平滑过渡带 (中心 x=0 处平缓过渡) — lateral smoothing band (gentle transition at x=0)
          const blend = Math.max(0.0, Math.min(1.0, (x + 0.9) / 1.8));
          const smoothBlend = blend * blend * (3.0 - 2.0 * blend);
          let localZ = leftWave * (1.0 - smoothBlend) + rightWave * smoothBlend;

          // 叠加细微的自然荒原旷野微地形质感 (波长较长的平缓地貌) — superimposed fine natural-wilderness micro-terrain texture (long-wavelength gentle relief)
          // G28-fix：微地形幅值随浪高设置自适应，否则 H 调小时固定 40mm 微项反成主波形 — G28-fix: the micro-terrain amplitude adapts to the wave-height setting, otherwise with small H the fixed 40mm micro term becomes the dominant waveform
          const microAmp = Math.min(0.04, u.amp * 0.6);
          localZ += microAmp * Math.sin(0.35 * k_y * dy + x * 0.4);

          // 边缘平缓渐变融入外围自然地表 — edges blend smoothly into the surrounding natural terrain
          const absX = Math.abs(x);
          if (absX > 4.5) {
            const edgeFade = Math.max(0.0, Math.min(1.0, (7.5 - absX) / 3.0));
            localZ *= edgeFade;
          }

          z_road += localZ;
        }
      }
    }

    return { z_road, isKerb, isBump, isPothole, isRamp, mu };
  }

  getRoadSlope(x, y) {
    let dz_dy = 0;
    let dz_dx = 0;

    if (this.scenario === "slope_climb") {
      dz_dy = this.params.grade || 0.18;
    } else if (this.undulatingSections && this.undulatingSections.length) {
      for (const u of this.undulatingSections) {
        if (y >= u.y0 && y <= (u.y0 + u.len)) {
          const dy = y - u.y0;
          const k_y = (2.0 * Math.PI) / Math.max(1.5, u.wavelength);
          const phaseRad = (u.phaseDeg !== undefined ? u.phaseDeg : 180) * (Math.PI / 180);

          const dLeft_dy = u.amp * k_y * Math.cos(k_y * dy);
          const dRight_dy = u.amp * k_y * Math.cos(k_y * dy + phaseRad);
          const blend = Math.max(0.0, Math.min(1.0, (x + 0.9) / 1.8));
          const smoothBlend = blend * blend * (3.0 - 2.0 * blend);

          dz_dy += dLeft_dy * (1.0 - smoothBlend) + dRight_dy * smoothBlend;

          const leftWave = u.amp * Math.sin(k_y * dy);
          const rightWave = u.amp * Math.sin(k_y * dy + phaseRad);
          if (Math.abs(x) <= 1.2) {
            dz_dx += (rightWave - leftWave) / 1.8;
          }
        }
      }
    }

    const slopeAngle = Math.atan(dz_dy);
    const slopePct = dz_dy * 100;
    const isUphill = dz_dy > 0.04;
    const isDownhill = dz_dy < -0.04;
    const elev = this.getRoadElevation(x, y).z_road;
    const isSummit = Math.abs(dz_dy) <= 0.04 && elev > 0.08;
    const isTrough = Math.abs(dz_dy) <= 0.04 && elev < -0.08;

    return {
      dz_dy,
      dz_dx,
      slopeGrade: dz_dy,
      slopeAngle,
      slopePct,
      isUphill,
      isDownhill,
      isSummit,
      isTrough
    };
  }

  getMu(x, y) {
    return this.getRoadElevation(x, y).mu;
  }
}

// -------------------- CIRCUIT TRACK SPLINE & PROFILING --------------------
class CircuitPath extends TrackPath {
  constructor(waypoints, mu = 1.35, aggressiveness = 1.0) {
    super();
    this.mu = mu;
    this.aggressiveness = aggressiveness || 1.0;
    this.pts = [];
    this.waypoints = waypoints;
    
    // 1. Closed-Loop Catmull-Rom Spline with rich telemetry interpolation
    const N = waypoints.length;
    for(let i = 0; i < N; i++){
      const p0 = waypoints[(i - 1 + N) % N];
      const p1 = waypoints[i];
      const p2 = waypoints[(i + 1) % N];
      const p3 = waypoints[(i + 2) % N];
      
      const dist = Math.hypot(p2.x - p1.x, p2.y - p1.y);
      const steps = Math.max(4, Math.ceil(dist / 2.0)); // 2 meters step
      for(let s = 0; s < steps; s++){
        const t = s / steps, t2 = t*t, t3 = t2*t;
        const x = 0.5 * ((2*p1.x) + (-p0.x + p2.x)*t + (2*p0.x - 5*p1.x + 4*p2.x - p3.x)*t2 + (-p0.x + 3*p1.x - 3*p2.x + p3.x)*t3);
        const y = 0.5 * ((2*p1.y) + (-p0.y + p2.y)*t + (2*p0.y - 5*p1.y + 4*p2.y - p3.y)*t2 + (-p0.y + 3*p1.y - 3*p2.y + p3.y)*t3);
        const v_target = (p1.v_target || 45.0) * (1 - t) + (p2.v_target || 45.0) * t;
        
        this.pts.push({ 
          x, y,
          v_max: v_target,
          turn: p1.turn || "",
          turnZh: p1.turnZh || p1.turn || "",
          turnApex: (s === 0 ? !!p1.turnApex : false),
          sector: p1.sector || 1,
          gear: p1.gear || 3,
          isDRS: !!p1.isDRS,
          kerbSide: p1.kerbSide || null,
          runoff: p1.runoff || null,
          brakingBoard: (s === 0 ? (p1.brakingBoard || null) : null)
        });
      }
    }
    
    const P = this.pts.length;
    this.totalPoints = P;
    
    // 2. Heading, Normal & Cumulative Arc-Length Distance
    let totalDist = 0;
    for(let i = 0; i < P; i++){
      const prev = this.pts[(i - 1 + P) % P];
      const curr = this.pts[i];
      const next = this.pts[(i + 1) % P];
      
      const dx = next.x - prev.x;
      const dy = next.y - prev.y;
      curr.heading = Math.atan2(-dx, dy);
      curr.nx = Math.cos(curr.heading);
      curr.ny = Math.sin(curr.heading);
      
      const stepDist = Math.hypot(curr.x - prev.x, curr.y - prev.y);
      totalDist += stepDist;
      curr.s = totalDist;
    }
    this.totalLength = totalDist;
    
    // 3. Smooth Curvature Calculation across all points (7-point span)
    for(let i = 0; i < P; i++){
      const prev = this.pts[(i - 4 + P) % P];
      const curr = this.pts[i];
      const next = this.pts[(i + 4) % P];
      
      const ds = Math.hypot(next.x - prev.x, next.y - prev.y);
      let dpsi = next.heading - prev.heading;
      while(dpsi > Math.PI) dpsi -= 2 * Math.PI;
      while(dpsi < -Math.PI) dpsi += 2 * Math.PI;
      
      curr.curvature = dpsi / (ds + 1e-6);
    }
    
    // 3b. G22 赛车线：外-内-外目标线偏移（激进走线——吃满路肩、追求圈速）。
    //     ── G21 原实现的两个根因缺陷（scratch/diag_*.cjs 三脚本实测确认）────
    //     (a) 弯道分段碎片化：退出阈值 0.6× 太紧，T1-T4 蜗牛弯被切成 4 段，每段
    //         独立跑一遍完整「外侧→弯心→外侧」剖面；且存在弧长 15m/转角 3.6°
    //         的样条噪声伪弯段。实测 15 段中 5 段总转角 <25°。
    //     (b) apexOff/outOff 幅度只由赛道宽度与 severity 决定，完全不随段长缩放
    //         ⇒ 弧长仅 74m 的短弯也被要求做 ~10m 横向摆动，实测目标线折角
    //         0.806 m/m = 38.9°，物理上不可跟 → 车把一个弯扭成好几个弯。
    //     ── 修复 ────────────────────────────────────────────────────────────
    //     (1) 退出阈值放宽到 0.30×，减少同一物理弯被局部曲率回落切碎；
    //     (2) 丢弃总转角 < MIN_TURN_DEG 的伪弯段（样条噪声不是弯）；
    //     (3) 偏移剖面改【弧长参数化】并向相邻直道两侧各延伸 EXT_M —— 真实车手
    //         在入弯前的直道上就已靠外侧，横向移动的距离来自直道而非弯内。
    //         这是把折角降下来的关键：靠加长行程，而不是靠砍幅度，故仍能吃满路肩；
    //     (4) 幅度按有效长度限幅。四段余弦剖面的峰值斜率解析式为
    //         peak|d(offset)/ds| = 2π·(apexOff+outOff)/L_eff
    //         （实测校验：L=74m、A=10m → 0.85 m/m，与量到的 0.806 吻合），
    //         故反解 A ≤ SLOPE_MAX·L_eff/(2π)：短弯自动少摆、长弯吃满。
    //     (5) 窗口重叠时用「弯心权重」交叉淡入淡出，而非硬切归属，保证符号连续。
    //     注：与赛道宽度无关。hw_m=4.9（总宽 9.8m，比真实 F1 赛道 13~15m 更窄），
    //         且幅度由 hw_m 推导 —— 加宽赛道只会放大摆幅、扭得更凶。
    //     ── 3b. G22 racing line: outside-inside-out target-line offset (aggressive line — ride the kerbs, chase lap time).
    //     Two root defects of the G21 implementation (confirmed by three scratch/diag_*.cjs measurement scripts):
    //     (a) corner fragmentation: the exit threshold 0.6x was too tight, splitting the T1-T4 snail corners into 4 pieces, each running the full outside->apex->outside profile; plus spline-noise pseudo-corners (15m arc / 3.6° turn) — 5 of the 15 measured segments turned less than 25° in total.
    //     (b) apexOff/outOff magnitude depended only on track width and severity, never scaled with segment length -> even a 74m short bend was asked for ~10m of lateral swing; measured target-line kink was 0.806 m/m = 38.9°, physically unfollowable — the car wiggled one corner into several.
    //     Fixes: (1) relax the exit threshold to 0.30x so one physical corner is not chopped by local curvature dips; (2) drop pseudo-corners whose total turn < MIN_TURN_DEG (spline noise is not a corner); (3) make the offset profile arc-length parametrized and extend it EXT_M into the adjacent straights — a real driver is already outside on the straight before corner entry, so lateral travel comes from the straight, not from inside the corner (the key to reducing the kink angle: lengthen the travel instead of cutting the magnitude, so the kerbs are still fully used); (4) clamp magnitude by effective length: the four-segment cosine profile's peak-slope closed form is peak|d(offset)/ds| = 2π(apexOff+outOff)/L_eff (verified: L=74m, A=10m -> 0.85 m/m, matching the measured 0.806), so back-solve A <= SLOPE_MAX*L_eff/(2π) — short bends swing less, long bends use full kerb; (5) where windows overlap, cross-fade with apex weighting instead of hard ownership, keeping the sign continuous.
    //     Note: independent of track width. hw_m=4.9 (total width 9.8m, narrower than real F1 tracks at 13-15m), and magnitude is derived from hw_m — widening the track only amplifies the swing.
    const HW_M = 4.9, KERB_M = 1.35;
    const CURV_ENTER = 0.0045;              // 进入弯道阈值（R<222m） — corner-entry threshold (R<222m)
    const CURV_EXIT = CURV_ENTER * 0.30;    // 退出阈值（R<740m）——迟滞放宽防切碎 — exit threshold (R<740m); hysteresis relaxed to prevent chopping
    const MIN_TURN_DEG = 25.0;              // 段总转角下限，低于此为样条噪声 — minimum total turn per segment; below this it is spline noise
    const EXT_M = 55.0;                     // 剖面向两侧直道延伸（横向行程的真正来源） — profile extends into the straights on both sides (the true source of lateral travel)
    const SLOPE_MAX = 0.24;                 // |d(offset)/ds| 上限 m/m（≈13.5° 折角） — |d(offset)/ds| cap in m/m (≈13.5° kink angle)
    this.hw_m = HW_M; this.kerb_m = KERB_M;
    for (let i2 = 0; i2 < P; i2++) { this.pts[i2].offset = 0; this.pts[i2].cornerId = -1; }

    // (1) 曲率分弯（带迟滞） — split corners by curvature (with hysteresis)
    const rawSegs = [];
    let ci = 0;
    while (ci < P) {
      if (Math.abs(this.pts[ci].curvature) > CURV_ENTER) {
        let cj = ci + 1;
        while (cj < P && Math.abs(this.pts[cj].curvature) > CURV_EXIT) cj++;
        rawSegs.push([ci, cj - 1]);
        ci = cj;
      } else ci++;
    }
    // (2) 段总转角 = 曲率沿弧长积分；弯心侧由符号定（右弯 curv<0 → 弯心在右 +） — segment total turn = curvature integrated along arc length; apex side decided by sign (right-hand corner curv<0 -> apex on the right, +)
    const turnOf = (i0, i1) => {
      let t = 0;
      for (let k = i0; k < i1; k++) t += this.pts[k].curvature * (this.pts[k + 1].s - this.pts[k].s);
      return t;
    };
    const cornerSegs = [];
    for (const sg of rawSegs) {
      if (Math.abs(turnOf(sg[0], sg[1])) * 180 / Math.PI >= MIN_TURN_DEG) cornerSegs.push(sg);
    }
    this.cornerSegsDropped = rawSegs.length - cornerSegs.length;

    // (3)(4)(5) 逐段生成偏移剖面，交叉淡入淡出累加 — build the offset profile segment by segment, accumulating with cross-fades
    const TL = this.totalLength;
    const offNum = new Array(P).fill(0), offDen = new Array(P).fill(0);
    const cidW = new Array(P).fill(-1), cidBest = new Array(P).fill(-1);
    let cid = 0;
    for (const [i0, i1] of cornerSegs) {
      const s0 = this.pts[i0].s, s1 = this.pts[i1].s;
      const wA = s0 - EXT_M, wB = s1 + EXT_M;              // 延伸后的窗口 — window after extension
      const L_eff = Math.max(1.0, wB - wA);
      const sumCurv = turnOf(i0, i1);
      const apexDir = (sumCurv < 0) ? 1 : -1;
      const avgCurvAbs = Math.abs(sumCurv) / Math.max(1, i1 - i0 + 1) / 2.0;
      const severity = Math.min(1, avgCurvAbs / 0.010);     // 弯越急 → 越贴路肩/越外放 — the tighter the corner, the more the line hugs the kerb / opens outward
      // 激进口径：弯心骑满路肩内侧（HW+0.65·KERB≈5.78m），入/出弯贴外侧沥青边 — aggressive convention: apex rides fully on the kerb inner edge (HW+0.65·KERB≈5.78m), entry/exit hug the outer asphalt edge
      const sevF = 0.62 + 0.38 * severity;
      let apexOff = (HW_M + 0.65 * KERB_M) * sevF;
      let outOff = (HW_M - 0.10) * sevF;
      // (4) 折角预算反解幅度上限 — (4) back-solve the magnitude cap from the kink-angle budget
      const ampMax = SLOPE_MAX * L_eff / (2 * Math.PI);
      const ampScale = Math.min(1.0, ampMax / Math.max(0.01, apexOff + outOff));
      apexOff *= ampScale; outOff *= ampScale;

      for (let k = 0; k < P; k++) {
        // 环状窗口：s 可能跨起终点，取落在 [wA,wB] 内的那个镜像 — wraparound window: s may cross start/finish, take the mirrored copy that lands in [wA,wB]
        const sk = this.pts[k].s;
        let cand = null;
        for (let m = -1; m <= 1; m++) {
          const c = sk + m * TL;
          if (c >= wA && c <= wB) { cand = c; break; }
        }
        if (cand === null) continue;
        const phi = (cand - wA) / L_eff;
        // 四段余弦（C¹）：0 → −outOff → +apexOff → −outOff → 0，段边界归零不跳变 — four-segment cosine (C1): 0 -> -outOff -> +apexOff -> -outOff -> 0; back to zero at segment boundaries, no jumps
        let prof;
        if (phi < 0.25) { const t = phi / 0.25; prof = -outOff * (1 - Math.cos(Math.PI * t)) / 2; }
        else if (phi < 0.5) { const t = (phi - 0.25) / 0.25; prof = -outOff + (apexOff + outOff) * (1 - Math.cos(Math.PI * t)) / 2; }
        else if (phi < 0.75) { const t = (phi - 0.5) / 0.25; prof = apexOff - (apexOff + outOff) * (1 - Math.cos(Math.PI * t)) / 2; }
        else { const t = (phi - 0.75) / 0.25; prof = -outOff * (1 + Math.cos(Math.PI * t)) / 2; }
        // (5) 弯心权重：phi=0.5 处 1，窗口两端 0 —— 重叠段之间平滑过渡 — (5) apex weighting: 1 at phi=0.5, 0 at the window ends — smooth transitions between overlapping segments
        const w = Math.max(0, 1 - Math.abs(phi - 0.5) * 2);
        if (w <= 0) continue;
        offNum[k] += apexDir * prof * w;
        offDen[k] += w;
        if (w > cidBest[k]) { cidBest[k] = w; cidW[k] = cid; }
      }
      cid++;
    }
    for (let k = 0; k < P; k++) {
      this.pts[k].offset = offDen[k] > 1e-9 ? offNum[k] / offDen[k] : 0;
      this.pts[k].cornerId = offDen[k] > 1e-9 ? cidW[k] : -1;
    }
    this.cornerCount = cid;
    // 移动平均平滑（防段间微跳与离散噪声），环状窗口 7 点 — moving-average smoothing (guards against inter-segment micro-jumps and discrete noise), wraparound 7-point window
    const offRaw = this.pts.map(p => p.offset);
    for(let i3 = 0; i3 < P; i3++){
      let s = 0;
      for(let w2 = -3; w2 <= 3; w2++) s += offRaw[(i3 + w2 + P) % P];
      this.pts[i3].offset = s / 7;
    }
    // (6) 斜率硬限幅。步骤 (4) 的幅度预算只对【单段】剖面成立（峰值斜率
    //     2π·A/L_eff）；步骤 (5) 的交叉淡入淡出在窗口重叠区会合成出超出该解析界
    //     的斜率（实测 0.291 > 0.24）。故对 offset 数组做前向+后向双向限幅，
    //     使 |d(offset)/ds| ≤ SLOPE_MAX 【恒成立】，不依赖剖面形状的假设。
    //     — (6) hard slope clamp. The magnitude budget in step (4) only holds for a single-segment profile (peak slope 2π·A/L_eff); the cross-fade in step (5) can synthesize slopes above that analytic bound in overlapping windows (measured 0.291 > 0.24). Therefore clamp the offset array forward+backward so |d(offset)/ds| <= SLOPE_MAX holds unconditionally, independent of profile-shape assumptions.
    for (let pass = 0; pass < 6; pass++) {
      let changed = false;
      for (let k = 0; k < P - 1; k++) {
        const lim = SLOPE_MAX * Math.max(1e-6, this.pts[k + 1].s - this.pts[k].s);
        const d = this.pts[k + 1].offset - this.pts[k].offset;
        if (d > lim) { this.pts[k + 1].offset = this.pts[k].offset + lim; changed = true; }
        else if (d < -lim) { this.pts[k + 1].offset = this.pts[k].offset - lim; changed = true; }
      }
      for (let k = P - 1; k > 0; k--) {
        const lim = SLOPE_MAX * Math.max(1e-6, this.pts[k].s - this.pts[k - 1].s);
        const d = this.pts[k].offset - this.pts[k - 1].offset;
        if (d > lim) { this.pts[k - 1].offset = this.pts[k].offset - lim; changed = true; }
        else if (d < -lim) { this.pts[k - 1].offset = this.pts[k].offset + lim; changed = true; }
      }
      if (!changed) break;
    }
    for(let i4 = 0; i4 < P; i4++){
      const pt = this.pts[i4];
      pt.refX = pt.x + pt.nx * pt.offset;
      pt.refY = pt.y + pt.ny * pt.offset;
    }

    // 3c. G22 赛车线自身的切向 / 曲率 / 法向 / 弧长（与中心线同口径的 7 点跨距）。
    //     ★ 根因修复核心：控制器此前用【中心线】的 heading/curvature 去追【赛车线】的
    //       横向目标，航向项增益 1.0 对横向项 ~0.03 形成 30:1 压制 ⇒ 车被拉回中线。
    //       这三个量让赛车线成为一等公民，drive() 的三个参考量得以同源。
    //       — 3c. G22 racing line's own tangent / curvature / normal / arc length (same 7-point span convention as the centerline). Core root-cause fix: the controller previously tracked the racing line's lateral target using the centerline's heading/curvature; the heading gain 1.0 suppressed the ~0.03 lateral term 30:1, dragging the car back to the centerline. These three quantities make the racing line a first-class citizen and give drive() consistent reference inputs.
    for (let i5 = 0; i5 < P; i5++) {
      const pv = this.pts[(i5 - 4 + P) % P], nx5 = this.pts[(i5 + 4) % P];
      this.pts[i5].refHeading = Math.atan2(-(nx5.refX - pv.refX), nx5.refY - pv.refY);
    }
    let sRef = 0;
    for (let i6 = 0; i6 < P; i6++) {
      const pv = this.pts[(i6 - 1 + P) % P], cu = this.pts[i6];
      sRef += Math.hypot(cu.refX - pv.refX, cu.refY - pv.refY);
      cu.sRef = sRef;
    }
    this.totalLengthRef = sRef;
    for (let i7 = 0; i7 < P; i7++) {
      const pv = this.pts[(i7 - 4 + P) % P], nx7 = this.pts[(i7 + 4) % P];
      const ds = Math.hypot(nx7.refX - pv.refX, nx7.refY - pv.refY);
      let dpsi = nx7.refHeading - pv.refHeading;
      while (dpsi > Math.PI) dpsi -= 2 * Math.PI;
      while (dpsi < -Math.PI) dpsi += 2 * Math.PI;
      this.pts[i7].refCurvature = dpsi / (ds + 1e-6);
      this.pts[i7].refNx = Math.cos(this.pts[i7].refHeading);
      this.pts[i7].refNy = Math.sin(this.pts[i7].refHeading);
    }
    this._hintCtrl = 0;   // getLookahead 最近点搜索的 hint 缓存 — hint cache for the getLookahead nearest-point search
    this._hintRoad = 0;   // getRoadElevation 的 hint 缓存（每子步 4 轮，热路径） — hint cache for getRoadElevation (4 wheels per substep, hot path)
    
    // 4. Physical Aerodynamic Speed Envelope & Friction Limits
    // G21（2026-09-01）push 升级：横向包络 0.72μ→0.85μ，制动 4.8→5.5，加速 5.2→5.8（真实车手探极限口径）
    // G22：横向包络改用【赛车线曲率 refCurvature】而非中心线曲率 —— 车跑的是赛车线，
    //      可用侧向加速度由赛车线的真实半径决定。弯心处赛车线半径更大 → 曲率更小
    //      → 自然得到更高弯速，这就是「赛车线圈速奖励」的物理来源，无需再拍一个
    //      经验百分比（原 5b 的 +4% 已删）；而换线过渡区 refCurvature 偏大 → 自动压速，
    //      也是正确的（横向移动本身要吃掉一部分抓地力预算）。
    //      — G21 (2026-09-01) push upgrade: lateral envelope 0.72μ→0.85μ, braking 4.8→5.5, acceleration 5.2→5.8 (real-driver limit-probing convention). G22: the lateral envelope now uses the racing-line curvature refCurvature instead of the centerline curvature — the car drives the racing line, and the usable lateral acceleration is decided by the racing line's real radius; a larger racing-line radius at the apex -> smaller curvature -> naturally higher cornering speed. That is the physics behind the "racing-line lap-time bonus" (the empirical +4% from the old 5b is deleted); and in transition zones refCurvature is larger -> speed is automatically limited, which is also correct (the lateral shift itself consumes part of the grip budget).
    const vehType = (typeof window !== "undefined" && window.S && window.S.vehicleType) ? window.S.vehicleType : "formula";
    const v_top_kmh = (vehType === "formula") ? 335.0 : (vehType === "gt3" || vehType === "sport") ? 295.0 : (vehType === "kart") ? 140.0 : 245.0;
    const v_top_veh = v_top_kmh * (1000.0 / 3600.0);
    const aggrScale = Math.min(1.35, Math.max(0.65, this.aggressiveness));
    const a_lat_max = Math.max(4.5, this.mu * 9.81 * 0.85 * aggrScale);
    
    for(let i = 0; i < P; i++){
      const pt = this.pts[i];
      const kLine = (pt.refCurvature !== undefined) ? pt.refCurvature : pt.curvature;
      const curv_v = Math.sqrt(a_lat_max / (Math.abs(kLine) + 1e-5));
      pt.v_max = Math.min(v_top_veh, Math.min(pt.v_max, curv_v));
    }
    
    // 5. Backward & Forward Braking/Acceleration Passes
    // G21：push 口径——真实车手试探刹车点的包络上限（逐圈学习会在运行时继续压低） — G21: push convention — the envelope ceiling for a real driver probing the braking point (per-lap learning keeps lowering it at runtime)
    const a_brake = 5.5 * Math.min(1.2, Math.max(0.8, aggrScale));
    const a_accel = 5.8 * Math.min(1.2, Math.max(0.8, aggrScale));
    
    for(let pass = 0; pass < 60; pass++) {
      let maxDiff = 0;
      for(let i = P - 1; i >= 0; i--){
        const nextIdx = (i + 1) % P;
        const ds = Math.hypot(this.pts[nextIdx].x - this.pts[i].x, this.pts[nextIdx].y - this.pts[i].y);
        const v_limit = Math.sqrt(this.pts[nextIdx].v_max**2 + 2 * a_brake * ds);
        if(v_limit < this.pts[i].v_max){
          maxDiff = Math.max(maxDiff, this.pts[i].v_max - v_limit);
          this.pts[i].v_max = v_limit;
        }
      }
      if(maxDiff < 0.01) break;
    }

    for(let pass = 0; pass < 60; pass++) {
      let maxDiff = 0;
      for(let i = 0; i < P; i++){
        const prevIdx = (i - 1 + P) % P;
        const ds = Math.hypot(this.pts[i].x - this.pts[prevIdx].x, this.pts[i].y - this.pts[prevIdx].y);
        const v_limit = Math.sqrt(this.pts[prevIdx].v_max**2 + 2 * a_accel * ds);
        if(v_limit < this.pts[i].v_max){
          maxDiff = Math.max(maxDiff, this.pts[i].v_max - v_limit);
          this.pts[i].v_max = v_limit;
        }
      }
      if(maxDiff < 0.01) break;
    }

    // 5b. G22：原「弯点 +4% 速度奖励」已删除 —— 该经验加成是在补偿「速度包络用中心线
    //      曲率、但车实际跑赛车线」的口径不一致。步骤 4 改用 refCurvature 后，弯速奖励
    //      由赛车线的真实几何半径直接给出（弯心半径更大 → 曲率更小 → v_max 更高），
    //      物理自洽且逐点连续，不再需要按 cornerId 拍一个全局百分比。
    //      — 5b. G22: the original "corner +4% speed bonus" is deleted — that empirical bonus was compensating for the inconsistency of using centerline curvature for the speed envelope while the car actually runs the racing line. With step 4 switching to refCurvature, the cornering-speed bonus comes directly from the racing line's true geometric radius (larger apex radius -> smaller curvature -> higher v_max); it is physically self-consistent and pointwise continuous, so no global percentage per cornerId is needed anymore.
  }

  /* G22：带 hint 窗口的最近点搜索。
     性能根由：VehicleDynamics15DOF.step() 以 1000Hz 子步对【四个车轮】各调一次
     getRoadElevation，单帧约 16 子步 × 4 轮 = 64 次；旧的 O(P) 全量遍历在 P=2884 时
     是 ~18万次距离计算/帧。窗口 ±WIN 点（≈±80m）+ 两道退化护栏后降至 ~5千次。
     useRef=true 时对赛车线(refX/refY)搜索，false 对中心线(x/y)。
     护栏：① 命中窗口边界 ⇒ 可能真最近点在窗外；② 窗口内最近距离 > 25m ⇒
     车辆可能已传送/重生。任一命中则回退全量搜索，保证结果与旧实现一致。
  — G22: nearest-point search with a hint window. Why: VehicleDynamics15DOF.step() runs at 1000Hz substeps and calls getRoadElevation once per wheel (4 wheels), ~16 substeps per frame = 64 calls/frame; the old O(P) full scan at P=2884 was ~180k distance computations per frame. A ±WIN-point window (~±80m) plus two degradation guards brings it to ~5k. useRef=true searches the racing line (refX/refY), false the centerline (x/y). Guards: (1) the search hits a window boundary -> the true nearest point may be outside; (2) the nearest in-window distance > 25m -> the vehicle may have teleported/respawned. Either guard falls back to the full scan, guaranteeing identical results to the old implementation. */
  _nearestIdx(x, y, hint, useRef) {
    const P = this.pts.length;
    const WIN = 40;
    const px = p => (useRef ? p.refX : p.x), py = p => (useRef ? p.refY : p.y);
    if (typeof hint === "number" && hint >= 0 && hint < P && P > WIN * 2) {
      let best = -1, bd = Infinity, lo = -1, hi = -1;
      for (let k = -WIN; k <= WIN; k++) {
        const i = (hint + k + P) % P, p = this.pts[i];
        const d = (px(p) - x) ** 2 + (py(p) - y) ** 2;
        if (d < bd) { bd = d; best = i; }
        if (k === -WIN) lo = i;
        if (k === WIN) hi = i;
      }
      if (best !== lo && best !== hi && bd < 625.0) return best;
    }
    let best = 0, bd = Infinity;
    for (let i = 0; i < P; i++) {
      const p = this.pts[i];
      const d = (px(p) - x) ** 2 + (py(p) - y) ** 2;
      if (d < bd) { bd = d; best = i; }
    }
    return best;
  }

  /* G22：沿赛车线弧长前进 ld 米，返回目标点索引。
     旧实现名为 getLookahead 但只返回【最近点】，v 参数完全未用。现保留真弧长
     前进能力，但【默认关闭】——原因见 _leadDist。
  — G22: advance ld meters along the racing line's arc length and return the target point index. The old function named getLookahead only returned the nearest point and its v parameter was entirely unused. True arc-length advance is kept, but DEFAULT OFF — reason in _leadDist. */
  _advanceIdx(i, ld) {
    if (!(ld > 0)) return i;
    const P = this.pts.length;
    let acc = 0, k = i;
    for (let n = 0; n < P; n++) {
      const kn = (k + 1) % P;
      const a = this.pts[k], b = this.pts[kn];
      const seg = Math.hypot(b.refX - a.refX, b.refY - a.refY);
      if (acc + seg >= ld) return kn;
      acc += seg; k = kn;
    }
    return k;
  }

  /* G22：前视距离——【默认 0】，这是经过推导的结论而非遗漏。
     drive() 是 Stanley 型控制器：横向误差在前轴量 + 曲率前馈。该组合在等曲率
     弧上【精确】：车在线上且对齐时 e_ψ=0、e_y=0 ⇒ δ = -atan(WB·k)，正好等于所需。
     若再对 heading 施加弧长前视 ld，前视点切向相对车身已转过 k·ld，会与曲率
     前馈【重复计权】：k=0.02（R=50m）、ld=20m 时多打 23°，而真正需要的只有 3°。
     旧代码的 `+forward*(lookDist*0.22)` 平移就是这类重复计权（且平移方向跟车身
     航向而非赛道）。如确需预览，请显式传 leadM 并同步下调曲率前馈。
     — G22: look-ahead distance — DEFAULT 0, a derived conclusion, not an omission. drive() is a Stanley-type controller: lateral error measured at the front axle + curvature feedforward. That combination is EXACT on a constant-curvature arc: on-line and aligned, e_psi=0 and e_y=0 => δ = -atan(WB·k), exactly what is needed. Adding an arc-length look-ahead ld on top of heading would double-count with the curvature feedforward: the look-ahead point's tangent is already rotated by k·ld relative to the body — at k=0.02 (R=50m), ld=20m that over-steers by 23° when only 3° is required. The old `+forward*(lookDist*0.22)` shift was exactly such double-counting (and it shifted along the body heading, not the track). If preview is truly needed, pass leadM explicitly and reduce the curvature feedforward accordingly. */
  _leadDist(v, leadM, kAt) {
    if (typeof leadM !== "number" || !isFinite(leadM) || leadM <= 0) return 0;
    // 显式启用时：急弯自动少看（发卡弯 R≈25m 里看 30m 等于看到 70° 开外） — when explicitly enabled: look less on tight corners (looking 30m ahead in a R≈25m hairpin means staring 70°+ off course)
    const ldCurv = 0.75 / Math.max(0.004, Math.abs(kAt || 0));
    return Math.max(0, Math.min(45.0, Math.min(leadM, ldCurv)));
  }

  getLookahead(x, y, v, leadM) {
    // ★ G22：最近点对【赛车线】搜索 —— 控制基准必须与目标线同源。 — G22: the nearest point is searched on the racing line — the control reference must share the target line's frame.
    const closestIdx = this._nearestIdx(x, y, this._hintCtrl, true);
    this._hintCtrl = closestIdx;
    const pt = this.pts[closestIdx];

    // 两套横向误差，语义必须分开：
    //   crossTrackError —— 相对【中心线】，物理赛道边界与罚时判定用（赛道边缘
    //                      关于中心线对称，不能用赛车线量，否则弯心永远不触发）；
    //   lineError       —— 相对【赛车线】，控制用（法向也取赛车线法向 refNx/refNy）。
    //  — Two lateral errors whose semantics must stay separate: crossTrackError is relative to the CENTERLINE (physical track boundary & penalty-time judgement; the track edges are symmetric about the centerline, so the racing line must NOT be used here or the apex would never trigger); lineError is relative to the RACING LINE (used for control; the normal is also the racing-line normal refNx/refNy).
    const crossTrackError = (x - pt.x) * pt.nx + (y - pt.y) * pt.ny;
    const lineError = (x - pt.refX) * pt.refNx + (y - pt.refY) * pt.refNy;

    const ld = this._leadDist(v, leadM, pt.refCurvature);
    const lookIdx = this._advanceIdx(closestIdx, ld);
    const tp = this.pts[lookIdx];

    return {
      // ★ G22：三个控制参考量全部取自赛车线的前视点（修复前：heading/curvature
      //   取中心线、横向目标取赛车线，两条线互相打架） — G22: all three control references come from the racing line's look-ahead point (before the fix: heading/curvature came from the centerline while the lateral target came from the racing line — the two lines fought each other)
      targetHeading: tp.refHeading,
      targetCurvature: tp.refCurvature,
      targetSpeed: tp.v_max,
      crossTrackError: crossTrackError,
      lineError: lineError,
      // 以下 x/y/nx/ny 保持【中心线】语义不变（向后兼容：11-stages.js:3782 的
      // 转播机位用 tvPt.x/y/nx/ny 向赛道侧外推 22m，改成赛车线会偏移最多 5.8m） — G21/G22: the following x/y/nx/ny keep CENTERLINE semantics (backward compatibility: the broadcast cameras at 11-stages.js:3782 extrapolate tvPt.x/y/nx/ny 22m off the track side; switching to the racing line would shift them up to 5.8m)
      nx: pt.nx,
      ny: pt.ny,
      x: pt.x,
      y: pt.y,
      idx: closestIdx,
      // G21/G22：赛车线目标点与法向（外-内-外）、弯道索引（逐圈刹车学习用） — G21/G22: racing-line target point & normal (outside-inside-out) and corner index (for per-lap braking learning)
      offset: pt.offset || 0,
      refX: pt.refX,
      refY: pt.refY,
      refNx: pt.refNx,
      refNy: pt.refNy,
      cornerId: pt.cornerId !== undefined ? pt.cornerId : -1,
      lookIdx: lookIdx,
      leadM: ld,
      s: pt.s || 0,
      sRef: pt.sRef || 0,
      turn: pt.turn,
      turnZh: pt.turnZh,
      sector: pt.sector,
      gear: pt.gear || 3,
      isDRS: pt.isDRS,
      kerbSide: pt.kerbSide,
      runoff: pt.runoff,
      brakingBoard: pt.brakingBoard
    };
  }

  /* G30（2026-09-04）：参考线等距采样——MPC 预测时域用。从弧长 sStart 起、
     沿赛车线每 ds 取一个参考点，共 n 个（闭合赛道对 totalLength 取模）。
     返回字段与 pts/赛车线同源：x/y（赛车线）、heading、curvature（赛车线）、
     v_max、cornerId、isDRS、s。StraightPath/CirclePath 等无 refX 的路径回退中心线。
  — G30 (2026-09-04): equidistant sampling of the reference line for the MPC prediction horizon. Starting from arc length sStart, take one reference point every ds along the racing line, n points total (mod totalLength on closed circuits). Returned fields share the pts/racing-line sources: x/y (racing line), heading, curvature (racing line), v_max, cornerId, isDRS, s. Paths without refX (StraightPath/CirclePath etc.) fall back to the centerline. */
  samplePath(sStart, n, ds = 2.0) {
    const L = this.totalLength || 0;
    const out = [];
    if (L <= 0) return out;
    for (let k = 0; k < n; k++) {
      const s = (((sStart + k * ds) % L) + L) % L;
      const idx = this._idxAtS(s);
      const p = this.pts[idx];
      out.push({
        x: p.refX !== undefined ? p.refX : p.x,
        y: p.refY !== undefined ? p.refY : p.y,
        heading: p.refHeading !== undefined ? p.refHeading : p.heading,
        curvature: p.refCurvature !== undefined ? p.refCurvature : (p.curvature || 0),
        v_max: p.v_max,
        cornerId: p.cornerId !== undefined ? p.cornerId : -1,
        isDRS: !!p.isDRS,
        s
      });
    }
    return out;
  }

  /* 弧长 → pts 索引（pts[i].s 沿环单调递增；s ∈ [0, L) 环形归属）。
     二分 + 端部回绕：pts[0].s 为首段长（非 0），首段覆盖 s ∈ [0, pts[0].s)。
  — Arc length -> pts index (pts[i].s increases monotonically along the loop; s ∈ [0, L) wraps). Binary search + end wraparound: pts[0].s is the first segment's length (non-zero) and the first segment covers s ∈ [0, pts[0].s). */
  _idxAtS(s) {
    const pts = this.pts, N = pts.length;
    if (N === 0) return 0;
    // 首段 [0, pts[0].s) 归属：s 超过半段就近 pts[0]，否则回绕到 pts[N-1] — first-segment [0, pts[0].s) ownership: s beyond half a segment maps to pts[0], otherwise wraps to pts[N-1]
    if (s <= pts[0].s) return (s > pts[0].s * 0.5) ? 0 : N - 1;
    let lo = 0, hi = N - 1;
    while (lo < hi) {
      const mid = (lo + hi + 1) >> 1;
      if (pts[mid].s <= s) lo = mid; else hi = mid - 1;
    }
    return lo;
  }

  getRoadElevation(x, y) {
    // G22：热路径——VehicleDynamics15DOF.step() 以 1000Hz 子步对四轮各调一次，
    // 单帧 ~64 次。旧的 O(P) 全量遍历在 P=2884 时是 ~18万次距离计算/帧，
    // 改用 hint 窗口后降至 ~5千次。路面高程/路肩/砾石均关于【中心线】定义，
    // 故此处仍对中心线搜索（useRef=false），与 getLookahead 的赛车线基准分开。
    // G22: hot path — VehicleDynamics15DOF.step() calls this once per wheel per 1000Hz substep, ~64 calls/frame. The old O(P) full scan at P=2884 was ~180k distance computations per frame; the hint-window version brings it to ~5k. Road elevation / shoulder / gravel are all defined relative to the CENTERLINE, so the search stays on the centerline (useRef=false), separate from getLookahead's racing-line basis.
    const closestIdx = this._nearestIdx(x, y, this._hintRoad, false);
    this._hintRoad = closestIdx;

    const pt = this.pts[closestIdx];
    const ey = (x - pt.x) * pt.nx + (y - pt.y) * pt.ny;
    const absEy = Math.abs(ey);
    const hw = this.hw_m || 7.0; // G21：窄道——沥青半宽（新 4.9m，旧赛道对象回退 7.0） — G21: narrow lane — asphalt half-width (new 4.9m; older track objects fall back to 7.0)
    const kw = this.kerb_m || 1.35; // 1.35m FIA Kerb width
    const s = pt.s || (closestIdx * 2.0); // G22：用真实累计弧长（旧实现用 idx*2 估算） — G22: use the true accumulated arc length (the old implementation estimated idx*2)
    
    let z_road = 0.0;
    let isKerb = false;
    let kerbRatio = 0.0;
    let mu = this.mu || 1.35;
    
    const hasKerb = pt.kerbSide || (Math.abs(pt.curvature || 0) > 0.006);
    
    if (absEy > hw && absEy <= (hw + kw) && hasKerb) {
      isKerb = true;
      kerbRatio = (absEy - hw) / kw;
      // Smooth sinusoidal bevel slope (up to 38mm kerb crest)
      const z_bevel = 0.038 * Math.sin(kerbRatio * (Math.PI / 2.0));
      // High-frequency Rumble Strip sinusoidal serrations (wavelength 0.35m, amplitude 12mm)
      const lambda = 0.35;
      const z_rumble = 0.012 * Math.sin((Math.PI * s) / lambda)**2;
      z_road = z_bevel + z_rumble * Math.sin(kerbRatio * Math.PI);
      mu = 1.18; // Painted kerb friction coefficient
    } else if (absEy > (hw + kw)) {
      // Off-track grass / gravel apron
      z_road = -0.012 + 0.004 * Math.sin(s * 3.1) * Math.cos(ey * 2.0);
      mu = 0.78;
    } else {
      // Pristine track asphalt
      z_road = 0.0;
      mu = this.mu || 1.35;
    }
    
    return { z_road, isKerb, kerbRatio, mu, ey, s, pt };
  }
}

window.TrackPath = TrackPath;
window.CirclePath = CirclePath;
window.StraightPath = StraightPath;
window.CircuitPath = CircuitPath;



