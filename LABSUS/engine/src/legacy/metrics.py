"""指标结果模型 — 七状态状态机（设计文档 §P2-3、进展文档 P2-3）。

每个指标必须返回 `MetricResult`，其 status 为 ResultStatus 七状态之一：

    VALID / APPROXIMATE / NOT_APPLICABLE / NOT_IMPLEMENTED /
    SOLVER_FAILED / EQUILIBRIUM_FAILED / OUT_OF_RANGE

约定（禁止 None 空白）：
- value 为数值时 status 必须是 VALID / APPROXIMATE / OUT_OF_RANGE；
- value 为 None 仅允许 status ∈ {NOT_APPLICABLE, NOT_IMPLEMENTED,
  SOLVER_FAILED, EQUILIBRIUM_FAILED}，且 note 说明原因；
- 页面不得因 value=None 而显示空白/误以为已计算。

P2-3 指标状态机：
- VALID          输入满足几何/物理前提，数值可信
- APPROXIMATE    输入接近前提（如残差略超阈值），数值近似
- NOT_APPLICABLE 该指标对当前配置不适用（如无转向输入时 Ackermann）
- NOT_IMPLEMENTED 尚未实现或依赖后续阶段（显式占位，禁止伪造数值）
- SOLVER_FAILED  求解失败或输入缺失导致无法计算
- EQUILIBRIUM_FAILED 力平衡无法满足（P3 载荷指标使用）
- OUT_OF_RANGE   输入超出适用区间（如全行程残差超阈值）

M5 死代码清理（2026-09-20）：删除无任何消费方的 `MetricSet` 类与
`not_implemented()` 构造器（全仓 grep 仅命中定义行）。NOT_IMPLEMENTED 状态
本身仍是 `MetricResult.status` 的默认值，可直接构造，未丢失能力。

Metric-result model — the seven-state state machine (design doc §P2-3,
progress doc P2-3).

Every metric must return a `MetricResult` whose status is one of the seven
ResultStatus states:

    VALID / APPROXIMATE / NOT_APPLICABLE / NOT_IMPLEMENTED /
    SOLVER_FAILED / EQUILIBRIUM_FAILED / OUT_OF_RANGE

Conventions (None-blanks are forbidden):
- when value is a number, status must be VALID / APPROXIMATE /
  OUT_OF_RANGE;
- value=None is allowed only with status ∈ {NOT_APPLICABLE,
  NOT_IMPLEMENTED, SOLVER_FAILED, EQUILIBRIUM_FAILED}, and note must
  explain the reason;
- the frontend must not render a blank (or treat it as computed) just
  because value=None.

P2-3 metric state machine:
- VALID           inputs satisfy the geometric/physical preconditions;
                  the number is trustworthy
- APPROXIMATE     inputs are close to the preconditions (e.g. a residual
                  slightly over threshold); the number is approximate
- NOT_APPLICABLE  the metric does not apply to the current configuration
                  (e.g. Ackermann with no steering input)
- NOT_IMPLEMENTED not yet implemented or dependent on a later stage
                  (explicit placeholder; fabricating a value is forbidden)
- SOLVER_FAILED   solver failure or missing input makes computation impossible
- EQUILIBRIUM_FAILED  force balance cannot be satisfied (used by P3 load metrics)
- OUT_OF_RANGE    inputs outside the applicable interval (e.g. a
                  full-travel residual over threshold)

M5 dead-code cleanup (2026-09-20): removed the `MetricSet` class and the
`not_implemented()` constructor, which had no consumers (a repo-wide grep
only hit their definition lines). The NOT_IMPLEMENTED status itself is
still the default value of `MetricResult.status`, can still be constructed
directly, so no capability was lost.
"""
from __future__ import annotations

from pydantic import BaseModel

from src.legacy.models import ResultStatus

_VALUE_STATES = {ResultStatus.VALID, ResultStatus.APPROXIMATE,
                 ResultStatus.OUT_OF_RANGE}


def ok(value: float, unit: str = "", note: str = "") -> MetricResult:
    """便捷构造：数值有效（VALID）。

    Convenience constructor: value is valid (VALID).
    """
    return MetricResult(key="", value=round(float(value), 6), unit=unit,
                        status=ResultStatus.VALID, note=note)


def not_applicable(key: str, unit: str = "", note: str = "") -> MetricResult:
    return MetricResult(key=key, value=None, unit=unit,
                        status=ResultStatus.NOT_APPLICABLE, note=note)


def solver_failed(key: str, unit: str = "", note: str = "") -> MetricResult:
    return MetricResult(key=key, value=None, unit=unit,
                        status=ResultStatus.SOLVER_FAILED, note=note)


class MetricResult(BaseModel):
    """单个指标结果：值 + 单位 + 状态 + 说明。

    A single metric result: value + unit + status + note.
    """
    key: str
    value: float | None = None
    unit: str = ""
    status: ResultStatus = ResultStatus.NOT_IMPLEMENTED
    note: str = ""

    def model_post_init(self, __context) -> None:
        if self.status in _VALUE_STATES and self.value is None:
            raise ValueError(
                f"metric '{self.key}': status {self.status.value} requires a value")
        if self.status not in _VALUE_STATES and self.value is not None:
            raise ValueError(
                f"metric '{self.key}': status {self.status.value} must not carry a value")
