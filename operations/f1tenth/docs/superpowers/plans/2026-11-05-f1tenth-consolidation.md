# F1TENTH AAC Consolidation Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement inline. The user requested direct execution.

**Goal:** Consolidate selected desktop/Nano work into AAC without changing running Nano or deleting original work.

**Architecture:** Canonical sources live in AAC domain folders. Root compatibility imports/config links preserve existing APIs; local data and environments remain linked, historical material is indexed separately.

**Tech Stack:** ROS2 Jazzy/Humble, Python, C++17, acados, Docker, SSH read-only inventory.

**Spec:** `docs/superpowers/specs/2026-11-05-f1tenth-consolidation-design.md`

## Global Constraints

- Preserve original desktop/Nano files and existing AAC nodes; never overwrite conflicting packages.
- Do not copy x86/ARM binaries, virtual environments, build outputs, credentials, caches, or nested Git metadata.
- Livox comes from Nano's real driver, not the desktop message-only subset.
- ROS1/offline packages are excluded from default colcon discovery.
- AGX remains a deployment target; no remote write or physical actuator output.

## Review Focus

- Broken symlinks or duplicate package discovery after relocating sources.
- Hidden dependencies on original temporary experiments or original source directories.
- Full Livox message compatibility and SDK dependency distinct from package discovery.
- Local data/environment links accidentally treated as portable AGX artifacts.
- Source/data loss or changes to preexisting AAC/Nano files.

### Task 1: Preserve and import sources

- [x] Capture original AAC tracked-file hashes and Git status; create layout integration checks first and observe missing sources.
- [x] Copy selected desktop modules and retrieve Nano sensor sources into staging, excluding build/cache/Git files.
- [x] Copy recent 3D localization and its two solver adapters; record file hashes and source origins.
- [x] Preserve older material in a separate archive index with colcon exclusion.

### Task 2: Make canonical paths usable

- [x] Install workspace marker, `tools.*`/config aliases, and local data/environment links without overwriting existing files.
- [x] Update imported path references and localization dependencies for the AAC location.
- [x] Add PC/Nano/AGX role records, a small documented entrypoint and Jazzy PC compose configuration.
- [x] Run layout checks, existing algorithm tests, and the copied localization suite; address relocation failures.

### Task 3: Verify and document

- [x] Verify manifest hashes, original-file preservation and link integrity.
- [x] Run Jazzy colcon discovery, reject duplicates/ROS1 leakage, and compile/test feasible C++ components in separate build output.
- [x] Add Korean entry documentation and archive index; record exact verification limits.
- [x] Obtain one independent final review as required by executing-plans, address material findings, and rerun affected checks.
