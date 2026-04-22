# 常见问题排查

## `package://...` 资源无法解析

排查步骤：

- 确认包存在且包含 `package.xml` + `CMakeLists.txt`。
- 设置 `URDF2MJCF_MODEL_PATH` 指向包含你的包的目录。
- 尝试 `urdf-to-mjcf-modelpath scan /path/to/workspace` 自动检测包。

相关代码：`core/package_resolver.py`、`cli/model_path.py`。

## 输出路径被拒绝

转换器有意拒绝将 MJCF 写入与源 URDF 相同的目录。

请使用单独的输出目录：

```bash
urdf-to-mjcf robot.urdf --output output_mjcf/robot.xml
```

## 网格处理在某些机器上失败

`pymeshlab`、`coacd`、`pycollada`、`trimesh` 等重度依赖对平台敏感。

排查步骤：

- 确认原生依赖已正确安装。
- 使用 `--skip-mesh-postprocess` 重新运行以隔离问题。
- 确认网格文件存在且可独立加载。

## GLB/GLTF 网格在 MuJoCo 中无法加载

MuJoCo 不原生支持 GLB/GLTF 格式。后处理流水线会自动将 GLB 文件转换为 OBJ。如果看到 `Unsupported mesh format: .glb`：

- 确保**没有**使用 `--skip-mesh-postprocess`，该选项会跳过网格转换。
- 确认已安装 `trimesh`（`pip install trimesh`）。

## 贡献者静态检查

```bash
uv run ruff format --check src/urdf_to_mjcf tests
uv run ruff check src/urdf_to_mjcf tests
uv run mypy src/urdf_to_mjcf tests
uv run pytest
```
