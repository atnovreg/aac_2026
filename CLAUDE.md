# AAC workspace

The repository is `<workspace>/src/aac_2026`. Keep the existing sensors, perception, navigation, planning, control, interface, system, docker and operations categories. Do not recreate tools/scripts/config compatibility trees at the workspace root.

- Source and current setup instructions: `README.md`, `operations/f1tenth/docs/F1TENTH_시작하기.md`.
- Imported F1TENTH uses ROS2 Jazzy. Original AAC uses Humble. Keep build/install environments separate.
- Edit sources here. Workspace `bag`, `tracks`, `results`, `vendor/acados` and hidden Python environments may link to older local assets. Preserve their targets.
- Preserve existing unrelated changes and upstream submodule revisions. Change only what the task needs.
- Keep controller mathematics, gains, calibration and hardware outputs unchanged during structural cleanup.
- Reuse existing code and standard library tools. Avoid speculative abstractions or dependencies.
- Verify affected behavior before claiming success. Layout checks do not prove dependencies, real-time performance or physical hardware operation.

Useful commands from this repository:

```bash
bash operations/f1tenth/run.sh check --source-only
bash operations/f1tenth/run.sh python -m control.acados_mpcc --help
bash operations/f1tenth/run.sh localization-test
```

`run.sh` changes to the workspace directory, so use absolute paths for test discovery or data arguments when needed. New hardware environments must rebuild their own SDKs, ROS packages, Python environments and solvers; do not copy desktop binaries to AGX.
