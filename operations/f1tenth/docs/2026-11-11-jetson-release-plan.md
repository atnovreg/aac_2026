# Jetson manual-control release plan

**Goal:** Publish the tested manual controller to the existing private GitHub
repository and verify a separate native AGX Orin deployment.

**Design:** Keep ROS Humble and the existing manual-only launcher. Add a setup
action that installs named apt dependencies, obtains the pinned VESC source only
when absent, and reuses the existing bounded build. Never reset an existing VESC
checkout. Keep servo output preview-only pending the hardware owner's work.
No startup service or automatic arming is added.

**Execution:**
1. Observe `manual.sh setup` fail on an empty workspace; implement setup and verify
   a cold x86 build plus rejection of an incompatible existing checkout.
2. Add a short GitHub/Jetson quickstart with sparse checkout, setup, safe preview,
   read-only check, bench-only drive, and the remaining hardware scope.
3. Deploy the same commit into a separate Jetson workspace, reuse already-installed
   dependencies, build natively, run the existing 18 tests and a read-only check.
4. Review, commit and push `codex/jetson-manual-20261111` to origin; verify remote SHA.

User authorization covers preparation, deployment and GitHub upload. Preserve the
dirty original PC checkout and existing Jetson source copy. The Jetson has no
GitHub credentials; transfer through its already-authorized SSH connection without
copying credentials, then point its Git remote at the existing private repository.
Future GitHub pulls require the operator's own GitHub authentication.
