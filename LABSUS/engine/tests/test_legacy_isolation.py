"""V1 冻结线隔离守卫（M5 结构整理，2026-09-20）。

背景：`src/legacy/` 收纳了 2026-08-30 之前的第一代实现（X 前 / Y 右 / Z 上
坐标血脉）。它**只服务门禁测试与历史对照**，生产路径必须全部走 v3 主线
（X 外侧 / Y 前 / Z 上）。一旦生产代码 import 了 legacy，就会出现两套坐标系
在同一调用链上混用——这正是 `INTENTIONAL_DIFFERENCES.md` 要防的事。

本测试用 AST 扫描 `src/` 下所有生产模块（排除 `src/legacy/` 自身）与
`server.py` / `scripts/`，禁止出现 `src.legacy` 导入。

English: V1 freeze-line isolation guard (M5 structural cleanup, 2026-09-20).
Background: `src/legacy/` holds the first-generation implementation (pre-2026-08-30,
X-front / Y-right / Z-up lineage). It serves ONLY gate tests and historical comparison;
all production paths must go through the v3 mainline (X-outboard / Y-forward / Z-up).
Once production code imports legacy, two coordinate systems get mixed inside one call
chain — exactly what `INTENTIONAL_DIFFERENCES.md` is meant to prevent.
This test AST-scans all production modules under `src/` (excluding `src/legacy/` itself)
plus `server.py` / `scripts/`, and forbids any `src.legacy` import.
"""
from __future__ import annotations

import ast
from pathlib import Path

ENGINE = Path(__file__).resolve().parents[1]
SRC = ENGINE / "src"
LEGACY = SRC / "legacy"


def _production_files() -> list[Path]:
    files = [p for p in SRC.rglob("*.py") if LEGACY not in p.parents]
    files += [ENGINE / "server.py"]
    files += [p for p in (ENGINE / "scripts").rglob("*.py")]
    return [p for p in files if p.exists() and "__pycache__" not in p.parts]


def test_production_never_imports_legacy():
    """生产模块不得 import src.legacy（含 `from src import legacy` 形式）。"""  # English: production modules must not import src.legacy (including the `from src import legacy` form).
    offenders: list[str] = []
    for p in _production_files():
        tree = ast.parse(p.read_text(encoding="utf-8"), filename=str(p))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for a in node.names:
                    if a.name == "src.legacy" or a.name.startswith("src.legacy."):
                        offenders.append(f"{p.relative_to(ENGINE)}:{node.lineno} import {a.name}")
            elif isinstance(node, ast.ImportFrom):
                mod = node.module or ""
                if mod == "src.legacy" or mod.startswith("src.legacy."):
                    offenders.append(f"{p.relative_to(ENGINE)}:{node.lineno} from {mod}")
                # 相对导入：src/api/... 里出现 `from ..legacy` 也要抓 — relative import: a `from ..legacy` inside src/api/... must also be caught
                if node.level and (mod == "legacy" or mod.startswith("legacy.")):
                    offenders.append(f"{p.relative_to(ENGINE)}:{node.lineno} from {'.' * node.level}{mod}")
    assert not offenders, "生产代码污染 V1 冻结线：\n  " + "\n  ".join(offenders)


def test_legacy_is_self_contained():
    """legacy 内部只允许依赖 legacy 自身 + v3 主线白名单（solver/mechanism/geometry/kandc/convention）。"""  # English: inside legacy, only self-dependencies plus the v3-mainline whitelist are allowed (solver/mechanism/geometry/kandc/convention).
    allow = ("src.legacy", "src.core.convention", "src.geometry",
             "src.metrics.kandc", "src.solver.mechanism")
    offenders: list[str] = []
    for p in LEGACY.rglob("*.py"):
        tree = ast.parse(p.read_text(encoding="utf-8"), filename=str(p))
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and node.module and node.module.startswith("src."):
                if not any(node.module == a or node.module.startswith(a + ".") for a in allow):
                    offenders.append(f"{p.name}:{node.lineno} from {node.module}")
            elif isinstance(node, ast.Import):
                for a in node.names:
                    if a.name.startswith("src.") and not any(
                            a.name == x or a.name.startswith(x + ".") for x in allow):
                        offenders.append(f"{p.name}:{node.lineno} import {a.name}")
    assert not offenders, "legacy 依赖越界：\n  " + "\n  ".join(offenders)


def test_legacy_modules_are_importable_and_unreachable_from_api():
    """legacy 可导入（门禁用），但 api 层不得经由任何路径可达它。"""  # English: legacy stays importable (for gate use), but the API layer must not reach it through any path.
    import importlib

    for mod in ("src.legacy.models", "src.legacy.metrics",
                "src.legacy.kinematics", "src.legacy.angles", "src.legacy.pose"):
        importlib.import_module(mod)

    # API 层源码级检查（运行时导入链已由上一个测试覆盖）
    # English: API-layer source-level check (the runtime import chain is already covered by
    # the AST test above). Wave-1 translation added English comments inside src/api/*.py that
    # legitimately mention "legacy" (e.g. v3models.py G31-P7 "the legacy scalars"), so a bare
    # word scan no longer means a freeze-line breach; check actual legacy references instead —
    # reachability is still fully guarded at AST level by test_production_never_imports_legacy.
    for p in (SRC / "api").rglob("*.py"):
        text = p.read_text(encoding="utf-8")
        for bad in ("src.legacy", "from ..legacy", "from .legacy", "import legacy",
                    "import src.legacy"):
            assert bad not in text, f"{p.name} 提及 legacy：冻结线不得进入 API 层 ({bad} found)"
