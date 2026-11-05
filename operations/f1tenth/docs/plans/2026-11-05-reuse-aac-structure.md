# Reuse AAC Structure Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans in this session.

**Goal:** Reuse existing AAC categories and eliminate parallel legacy source directories and duplicate bag naming.

**Architecture:** Import Python code through its actual AAC functional namespaces. Keep data outside the repository; use the original workspace `bag/` name. Move shared path utility, replay, archive, and documentation into existing system/operations categories.

**Spec:** User correction: merge bag/bags and maximize reuse of the original autonomous-driving skeleton. Proceed in place and preserve existing data, algorithms, submodules and hardware environments.

## Global Constraints
- Do not modify Nano/AGX, upstream submodules, control mathematics or selected tuning values.
- Preserve all raw data and original source files; remove only verified links/generated caches.
- No new top-level source categories or replacement legacy compatibility tree.
- Keep distinct Humble/Jazzy build outputs; this task does not merge ROS environments.

## Review Focus
- Fresh clones must initialize without old workstation directories.
- Collision detection must preserve existing data before mutations.
- Python subprocesses and Docker must use the same AAC module paths as their callers.
- Canonical bag link must resolve both on the host and inside Docker.
- Preservation manifests must record moves/changed hashes without erasing source provenance.

## Tasks
- [x] Add regression checks for canonical `bag`, direct imports and absence of legacy root aliases.
- [x] Move common utility to system/f1tenth; replay to system/f1tenth_gazebo_replay; archive/docs/entry point under operations/f1tenth.
- [x] Replace old tools.* imports and legacy config/script paths with actual AAC paths; update setup, Compose and wrappers.
- [x] Merge bag aliases without deleting data; remove validated legacy source links and empty root folders.
- [x] Update provenance, documentation and setup checks; run Python/wrapper/source verification and isolated clone-layout verification.
- [x] Commit and push the verified correction to the already authorized private repository.

Verification: 119 Python tests and one native Gazebo interpolation test passed. Host/Compose symlink data and fresh real data directories passed; 10 original bag metadata files unchanged. Reviewed Docker path handling; no physical hardware commands were sent. Original desktop source/data and Nano runtime remain preserved.
