# Troubleshooting

## `package://...` assets cannot be resolved

Checks:

- Verify the package exists and contains `package.xml` + `CMakeLists.txt`.
- Set `URDF2MJCF_MODEL_PATH` to the directory containing your packages.
- Try `urdf-to-mjcf-modelpath scan /path/to/workspace` to auto-detect packages.

Related code: `core/package_resolver.py`, `cli/model_path.py`.

## Output path rejected

The converter intentionally rejects writing the MJCF into the same directory as the source URDF.

Use a separate output tree:

```bash
urdf-to-mjcf robot.urdf --output output_mjcf/robot.xml
```

## Mesh processing fails on one machine only

Heavy dependencies such as `pymeshlab`, `coacd`, `pycollada`, and `trimesh` are platform-sensitive.

Checks:

- Verify native dependencies are properly installed.
- Re-run with `--skip-mesh-postprocess` to isolate the issue.
- Confirm the mesh file exists and is loadable independently.

## GLB/GLTF meshes not loading in MuJoCo

MuJoCo does not natively support GLB/GLTF format. The post-processing pipeline automatically converts GLB files to OBJ. If you see `Unsupported mesh format: .glb`:

- Ensure you are **not** using `--skip-mesh-postprocess`, which skips mesh conversion.
- Verify that `trimesh` is installed (`pip install trimesh`).

## Static checks for contributors

```bash
uv run ruff format --check src/urdf_to_mjcf tests
uv run ruff check src/urdf_to_mjcf tests
uv run mypy src/urdf_to_mjcf tests
uv run pytest
```
