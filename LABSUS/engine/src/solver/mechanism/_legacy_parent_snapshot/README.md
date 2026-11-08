# _legacy_parent_snapshot／父仓冻结快照归档说明

# _legacy_parent_snapshot — archive notes for the frozen parent-repo snapshot

> 状态：**已在 M1 里程碑完成死代码清理**（2026-09-06）。
> 原有的三个快照文件（`bench.py`, `from_legacy.py`, `v2adapter.py`）依赖父仓外部模块
> （`hardpoints`, `config`, `routes.solve`, `tire`），在 LABSUS 独立运行环境中不可导入。
> 经依赖审计确认全仓零入向调用后已安全移除。

> Status: **dead-code cleanup completed at the M1 milestone** (2026-09-06).
> The three original snapshot files (`bench.py`, `from_legacy.py`,
> `v2adapter.py`) depended on parent-repo external modules (`hardpoints`,
> `config`, `routes.solve`, `tire`) and were not importable in LABSUS's
> standalone runtime. After a dependency audit confirmed zero inbound
> calls repo-wide, they were safely removed.

## 历史参照与替代实现
## Historical references and replacement implementations

- **门禁基准（G1–G4 判据）**：原 `bench.py` 56 例矩阵及残差要求已全面在
  `engine/tests/test_s1_gate.py` 中用 LABSUS 原生 `PRO_POINTS` 与 `build_mechanism` 重建并持续验证。
- **角解与机构装配**：已由 `src/api/v3service.py` 和 `src/solver/mechanism/solver.py` 原生承接。
- **坐标与对拍基准**：参见根目录 `INTENTIONAL_DIFFERENCES.md` 与 `tests/test_w4_consistency.py`。

- **Gate benchmarks (the G1–G4 criteria)**: the original `bench.py`
  56-case matrix and residual requirements have been fully rebuilt in
  `engine/tests/test_s1_gate.py` using LABSUS-native `PRO_POINTS` and
  `build_mechanism`, and are continuously verified.
- **Angle solution and mechanism assembly**: now handled natively by
  `src/api/v3service.py` and `src/solver/mechanism/solver.py`.
- **Coordinate and comparison benchmarks**: see the root-level
  `INTENTIONAL_DIFFERENCES.md` and `tests/test_w4_consistency.py`.
