"""
Engine 隔离工作区 conftest —— 仅挂载 engine/ 到 sys.path。

English: engine-isolated workspace conftest — mounts only engine/ onto sys.path.


F-74：不再挂 engine/src。引擎内部与测试一律使用 `src.` 绝对导入根，
否则同一文件会被加载成两个模块对象（`src.geometry` vs `geometry`），
isinstance / 身份比较会静默失效。

English: F-74: engine/src is no longer mounted; engine internals and tests all use the
`src.` absolute import root. Otherwise the same file would be loaded as two distinct
module objects (`src.geometry` vs `geometry`) and isinstance / identity comparisons
would silently fail.
"""
import os
import sys

_engine_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _engine_root not in sys.path:
    sys.path.insert(0, _engine_root)
