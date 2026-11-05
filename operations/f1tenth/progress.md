# SDD ledger — plan: docs/superpowers/plans/2026-11-05-f1tenth-consolidation.md

Execution: inline on `chore/f1tenth-consolidation-20261105`. The user explicitly requested organizing AAC in place; original directories and the Nano runtime are preserved. No worktree is created because the requested deliverable is the existing AAC workspace.

Pre-flight: imports need `tools.*` and a marked `ros_ws` data root. Localization also needs `soft_runtime.py` and `solver_margin.py` from a separate temporary experiment. Nano has the full Livox driver while desktop has only message definitions; only the full driver is imported.

Baseline: AAC had a preexisting dirty `navigation/WildGS-SLAM` submodule. Preserve it and exclude it from all changes/staging.

Task 1: complete. Imported desktop/Nano/localization sources and Livox SDK; read-only remote hash verification checked 585 files with zero changes. Existing AAC original-file hashes are preserved (548 files).

Task 2: complete. Canonical aliases, data links, local environment reuse, separate Jazzy/CPU build directories and host roles are installed. Layout/setup tests 9/9, MPCC 25/25, Slash model 10/10, tuner 19/19, planning 5/5 and localization 14/14 passed. The relocation and collision checks were observed failing before implementation.

Task 3: in progress. 29 unique ROS2 packages discovered; Python parsed 213 source files and shell checked 21 imported scripts. The fresh native Python-reference export produced a 5.34077e-6 C++ command difference against a 1e-6 tolerance; the unchanged controller is being tested against the original native C-batch reference configuration. No numerical tolerance is relaxed. PC image build is in progress.

Task 3: complete. Built the new Jazzy PC image and all three mapping packages (Livox full driver, bag adapter, FAST-LIO). Gazebo plugin build/interpolation test and front-filter message/geometry test passed. Native C++ executables built; 9/10 tests passed. The core-parity difference is identical (5.34077e-6) when building the original pre-migration C++ sources with the same freshly generated artifacts, so the inherited numerical issue is recorded without modifying controller code or tolerances.

Final review: independent read-only review completed. Two important relocation issues were fixed: absolute Compose asset defaults, and matching Gazebo artifact build/test/plugin search paths. Both regression tests were observed RED then GREEN; all 11 layout/setup/wrapper tests passed afterward. The log-copy issue was treated as a material relocation regression and its source path was corrected. No review findings remain deferred.

Final scope: preserved Nano runtime and all original sources; retained local data/runtime links; no AGX deployment, real sensor run, actuator output, merge or push. Existing AAC Humble build/install/log remain unchanged. Final syntax validation parsed 218 Python files and checked 23 shell scripts. Verification results and baseline limitations are in verification.json and verification_logs/.

Follow-up relocation (user requested): moved `/home/irdl4/Workspace/ros_ws` directly to `/home/irdl4/ros2_ws`, without a compatibility directory at the old source path. Retargeted 22 of 131 examined symlinks with no newly broken links. Updated host alias, source/data paths, active documentation and Compose asset defaults. Container paths `/ros_ws` and `/ws` remain stable. Four additional original AAC host-path files have byte-exact backups recorded in the import manifest. Post-move preservation checks, all 11 layout/setup/wrapper tests, shell syntax and Compose bind sources passed.
