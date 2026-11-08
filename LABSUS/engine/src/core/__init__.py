"""核心层：坐标规范、V1 数据模型、七状态指标模型与 JSON 清洗工具。

导入约定（M5 统一）：引擎内部一律使用 `src.` 绝对导入根，sys.path 只挂
`engine/`，不再挂 `engine/src/`——否则同一文件会被加载成 `src.geometry` 与
`geometry` 两个模块对象，isinstance / 身份比较会静默失效（F-74）。

Core layer: coordinate conventions, the V1 data models, the seven-state
metric model, and JSON sanitization utilities.

Import convention (unified in M5): engine internals always use the `src.`
absolute import root, and sys.path only mounts `engine/`, never
`engine/src/` — otherwise the same file gets loaded as two distinct module
objects, `src.geometry` and `geometry`, and isinstance / identity
comparisons silently stop working (F-74).
"""
