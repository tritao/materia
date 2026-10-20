---
title: Documentation inventory
description: "Every tracked Markdown file in Materia tagged keep, merge, internal or delete, with a stability label per kit and the target site map (DOCS_PLAN phase D0)."
status: draft
---

# Documentation inventory

Phase D0 of [DOCS_PLAN.md](DOCS_PLAN.md). Surveyed 2026-10-21 on `main` at
24c72edba: 324 tracked `.md` files (`git ls-files '*.md'`, without vendored
code, plus `haxeon/README.md` and `haxeon/docs/*`). Untracked trees
(`editorkit/`, `docs/api/`) are not covered.

## Tags

| Tag | Meaning |
|---|---|
| **keep** | Reader-facing. It stays where it is (a kit README, a notice, a format reference) and the site links to it. |
| **merge** | Its reader-facing content belongs in a site page under `docs/`, named in the notes. A kit README tagged merge stays as a short README (what the kit is, how to build and test it) after the move. |
| **internal** | A plan, TODO, handoff, research note, review, measurement log or ADR. It stays for contributors and is never linked from the site (Principle 5). |
| **delete** | Stale or superseded. The notes say by what. |

Targets under `docs/concepts/` exist today. Targets under `docs/guides/`,
`docs/manual/`, `docs/reference/`, `docs/contributing/` and `docs/machines/`
are proposed pages; the [site map](#site-map) lists them.

## Repository root

| File | Tag | Notes |
|---|---|---|
| CONTRIBUTING.md | keep | DCO, licences, submodule rule. GitHub convention; links to `docs/contributing/` once it exists. |
| README.md | keep | **Stale.** Pitches "a native application stack for 3D, CAD, simulation and editor software", which does not match `docs/README.md` ("design, simulate, plan and run machines"). Layout table lists 11 of 23 kits (no MachineKit, MotionKit siblings, CncKit, CamKit, StockKit, ...). Run/watch/live-reload section → `docs/guides/build-from-source.md`. |
| TODO.md | delete | Item 1 done; item 2 (queued-trajectory pause) is restated in `motionkit/README.md:115`. Move it to a MotionKit TODO. |

## app

| File | Tag | Notes |
|---|---|---|
| README.md | merge | 830 lines of editor behaviour (hierarchy, inspector, travel controls, mission monitor, workers, scripted setups, inspection camera, hobber scene) → `docs/manual/` (one page per panel/mode). Mentions compat for "older exports" and the "former body/Parts hierarchy", against the no-legacy rule. |
| docs/project-ui-extension.md | merge | Protocol `materia.project-ui.v1` → `docs/reference/project-ui-extension.md`, how-to → `docs/guides/project-ui-extensions.md`. |
| tests/enclosure-guard/README.md | internal | Test fixture notes. |
| tests/performance/README.md | internal | 2,030-line measurement log titled "Editor performance gate". Gate commands → `docs/contributing/performance-gates.md`; rest stays. |
| tests/project-loading/README.md | internal | Fixture + dated timings. |
| tools/responsiveness-benchmark-2026-09-23.md | internal | One dated benchmark. |
| web/README.md | merge | Build/run → `docs/guides/web-build.md`; architecture (guest/host, memory contract) → `concepts/platform/web-and-native.md`. |

## animkit

| File | Tag | Notes |
|---|---|---|
| README.md | merge | Implementation moved to `haxeon/packages/animation`; README still documents the moved API. Trim to the SceneKit `SkinnedModel` adapter; character content → `concepts/modelling/specialized-models.md`. |
| TODO.md | internal | |
| assets/quaternius-ual/PROVENANCE.md | keep | Asset provenance and licence. |
| native/THIRD_PARTY.md | keep | Stub pointing to Haxeon's animation THIRD_PARTY; says legacy `vendor/` copies remain for VisionKit tooling, which keeps duplicate cgltf/ozz/stb vendors alive. |
| tools/ual/README.md | internal | Asset-pipeline tool notes. |

## automationkit

| File | Tag | Notes |
|---|---|---|
| README.md | merge | Facility/mission/fleet/traffic model → `concepts/execution/facilities-and-fleets.md` (already drafted from it). |

## bimkit

| File | Tag | Notes |
|---|---|---|
| README.md | merge | → `concepts/modelling/specialized-models.md` (BIM section); IFC export → `docs/guides/export-ifc.md`. |

## cadkit

| File | Tag | Notes |
|---|---|---|
| ARCHITECTURE.md | merge | ABI contract and dependency rule → `concepts/modelling/cad-documents.md` + `docs/reference/cadkit-abi.md`. |
| README.md | keep | **Self-contradicting:** says "the first configure builds the pinned vendored OCCT", then that a build never starts OCCT (prebuilt prefix, `CADKIT_BUILD_OCCT_FROM_SOURCE` off). Fix the first paragraph. |
| examples/modeling/README.md | merge | Mounting plate + procedural excavator → `docs/guides/cad-modeling.md` / tutorials. |
| examples/nativekit/README.md | keep | Short example note. |
| haxe/MODELING.md | merge | 495-line modelling API manual → `docs/guides/cad-modeling.md`; types → generated reference. |
| haxe/README.md | merge | HXI regeneration → `docs/contributing/`; Shape/query API → `docs/guides/cad-modeling.md`. |
| haxe/benchmarks/README.md | internal | |
| plans/CONSTRAINT_SOLVING.md | internal | Plan + progress log (C0–C5 done). |
| plans/SHEET_METAL.md | internal | Plan (SM0–SM20). |
| plans/TOPOLOGICAL_NAMING.md | internal | Plan (TN0–TN9 done). |

## camkit

| File | Tag | Notes |
|---|---|---|
| HOBBING.md | merge | → `docs/guides/hobbing-cam.md`. |
| README.md | merge | 2.5D CAM API → `docs/guides/cam-2.5d.md`; concepts already in `concepts/process/toolpaths.md`. |

## cnckit

| File | Tag | Notes |
|---|---|---|
| PLAN.md | delete | Self-described "historical CNC roadmap"; every phase checked. Superseded by `cnckit/README.md` and `toolpathkit/docs/ADR-001-toolpath-ir.md`. |
| README.md | merge | G-code import/export → `docs/guides/gcode.md`; printer dialect → `docs/guides/printer-gcode.md`. |
| tests/fixtures/README.md | internal | Fixture provenance. |

## collisionkit

| File | Tag | Notes |
|---|---|---|
| README.md | keep | Short, accurate. |
| native/THIRD_PARTY.md | keep | coal fork notice (two fixes not yet upstreamed). |
| plans/COLLISION.md | internal | Plan + log (CL2–CL9). |

## docs

| File | Tag | Notes |
|---|---|---|
| DOCS_PLAN.md | internal | The docs plan. |
| OPEN_CORE.md | keep | Licensing decision; later the source of `docs/products/licensing`. |
| README.md | keep | Site landing (splash). |
| concepts/README.md | keep | Concept index. Only concept file without front matter. |
| concepts/glossary.md | keep | |
| concepts/start/overview.md | keep | draft |
| concepts/start/life-of-a-machine.md | keep | draft |
| concepts/foundations/conventions.md | keep | draft |
| concepts/foundations/data-and-runtime.md | keep | draft |
| concepts/foundations/determinism.md | keep | draft |
| concepts/foundations/diagnostics.md | keep | draft |
| concepts/foundations/frames.md | keep | draft |
| concepts/foundations/identity.md | keep | draft |
| concepts/foundations/immutability.md | keep | draft |
| concepts/foundations/truth-and-observation.md | keep | draft |
| concepts/modelling/assemblies-and-mates.md | keep | draft |
| concepts/modelling/cad-documents.md | keep | draft |
| concepts/modelling/collision.md | keep | draft |
| concepts/modelling/machine-components.md | keep | draft |
| concepts/modelling/mechanical-truth.md | keep | draft |
| concepts/modelling/specialized-models.md | keep | draft |
| concepts/modelling/transmissions.md | keep | draft |
| concepts/motion/execution-plans.md | keep | draft |
| concepts/motion/kinematic-forest.md | keep | draft |
| concepts/motion/machine-axes.md | keep | draft |
| concepts/motion/paths-and-trajectories.md | keep | draft; first baked figure. |
| concepts/execution/controller-boundary.md | keep | draft |
| concepts/execution/facilities-and-fleets.md | keep | draft |
| concepts/execution/fidelity.md | keep | draft |
| concepts/execution/robot-runtime.md | keep | draft |
| concepts/execution/simulation-worlds.md | keep | draft |
| concepts/platform/editor-and-projects.md | keep | draft |
| concepts/platform/haxeon.md | keep | draft |
| concepts/platform/nativekit-and-ui.md | keep | draft |
| concepts/platform/rendering.md | keep | draft |
| concepts/platform/web-and-native.md | keep | draft |
| concepts/process/manufacturing-outputs.md | keep | draft |
| concepts/process/process-and-robots.md | keep | draft |
| concepts/process/stock.md | keep | draft |
| concepts/process/toolpaths.md | keep | draft |
| site/README.md | merge | Site build, CI and Workers settings → `docs/contributing/docs-site.md`. |
| tutorials/README.md | keep | |
| tutorials/gantry-picker.md | keep | draft |

## gearkit

| File | Tag | Notes |
|---|---|---|
| METROLOGY.md | merge | Gear inspection API → `docs/guides/gear-inspection.md`; idea → `concepts/modelling/specialized-models.md`. |
| TOOTH_DESIGN_PLAN.md | internal | Plan. No gearkit README exists: add one. |

## haxeon

Haxeon is its own project with its own docs. The site links Haxeon's
reader-facing docs from `concepts/platform/haxeon.md` and the reference
section; it does not copy them.

| File | Tag | Notes |
|---|---|---|
| README.md | keep | |
| docs/AARCH64_IR_MIGRATION_AUDIT.md | internal | Migration audit. |
| docs/ALLOCATION_CENSUS.md | internal | Measurement. |
| docs/ANDROID.md | keep | Android host. |
| docs/ARCHITECTURE.md | keep | |
| docs/ARRAY_ALLOCATION_PROFILE.md | internal | Rejected experiment record. |
| docs/BENCHMARKS.md | keep | How to run benchmarks. |
| docs/BOUNDS_CHECK_ELIMINATION.md | internal | Rejected experiment record. |
| docs/BUMP_ALLOCATION_PLAN.md | internal | Plan. |
| docs/BUMP_ALLOCATION_PROFILE.md | internal | Measurement. |
| docs/BYTES_ACCESS_PROFILE.md | internal | Rejected experiment record. |
| docs/CLI.md | keep | Project CLI reference. |
| docs/COMPILER_GC_MARKING.md | internal | Experiment, acceptance pending. |
| docs/C_HEADER_FFI.md | keep | FFI import reference (891 lines). |
| docs/DEVELOPMENT.md | keep | Contributor build/test for Haxeon. |
| docs/FASTA_OUTPUT_PROFILE.md | internal | Measurement. |
| docs/FASTA_PROFILE.md | internal | Rejected experiments. |
| docs/GC_ALLOCATION_PROFILE.md | internal | Measurement. |
| docs/GC_EMPTY_PAGE_RETENTION.md | internal | Accepted change record. |
| docs/GC_PROFILE.md | internal | 1,576-line historical log; banner says superseded by GC_EMPTY_PAGE_RETENTION. |
| docs/GENERIC_SPECIALIZATION.md | internal | Compiler invariants. |
| docs/HASHLINK_MODULE_RECLAMATION.md | internal | Runtime design record. |
| docs/HLB_DEBUG_SECTIONS.md | internal | Bytecode format detail. |
| docs/HLD3_DEBUG_PROTOCOL.md | keep | Debugger protocol reference. |
| docs/IMPLEMENTATION_BASELINE.md | delete | 2026-09-05 baseline recorded "before step 1" of the reload work, which has landed (module reclamation, patch transactions). Git history keeps it. |
| docs/INLINING_AND_VALUE_TYPES.md | internal | Stage 1 shipped (`HAXEON_INLINE`); stages 2–4 design. |
| docs/IR_OPPORTUNITY_CENSUS.md | internal | Measurement. |
| docs/JIT_ALLOCATION_INITIALIZATION.md | internal | Dropped experiment. |
| docs/JIT_BOXED_ALLOCATION.md | internal | Dropped experiment. |
| docs/JIT_BOXED_ALLOCATION_LAYOUT.md | internal | Experiment. |
| docs/JIT_INLINE_ALLOCATION.md | internal | Opt-in stage 1 record. |
| docs/JIT_NATIVE_PROPERTIES.md | internal | Runtime detail. |
| docs/JIT_REGISTER_ALLOCATION.md | internal | Design. |
| docs/JIT_REGISTER_ALLOCATION_PHASE0.md | internal | Measurement. |
| docs/JIT_REGISTER_ALLOCATION_RESULTS.md | internal | Decision record. |
| docs/JSON_WIRE.md | keep | User-facing builtin. |
| docs/LANGUAGE_SUPPORT.md | keep | Language subset reference. |
| docs/LIVE_PATCHING.md | keep | |
| docs/MAIN_COMPATIBILITY_HANDOFF.md | internal | Branch handoff (2026-10-09); delete once landed. |
| docs/MESSAGEPACK.md | keep | Wire layer reference. |
| docs/METADATA_OWNERSHIP.md | internal | Runtime ownership rules. |
| docs/NATIVE_MEMORY.md | keep | |
| docs/OBJECT_ALLOCATION_PROFILE.md | internal | Measurement. |
| docs/OPTIMIZATION_NEXT_PLAN.md | delete | Executed; outcome in OPTIMIZATION_NEXT_RESULTS.md, and its own banner marks its hypothesis superseded. |
| docs/OPTIMIZATION_NEXT_RESULTS.md | internal | Results. |
| docs/PATCH_TRANSACTIONS.md | internal | Runtime design. |
| docs/PRAGTICAL_CONVERSION_PLAN.md | internal | Plan (2026-09-05); fixtures still in tests. |
| docs/PROFILING.md | keep | Profiling workflow. |
| docs/RPC.md | keep | Typed RPC reference. |
| docs/SCOPED_PLACES.md | internal | Deferred proposal. |
| docs/SEMANTIC_TYPE_CONTRACT.md | keep | Language semantics. |
| docs/SHADERS.md | internal | Deferred plan. |
| docs/STRUCTS.md | internal | Design, not implemented. |
| docs/STRUCTURAL_EQUALITY.md | keep | User-facing builtin. |
| docs/UTEST_UPSTREAM_COMPATIBILITY.md | internal | Compatibility baseline. |
| docs/WAMR_PROFILE_HANDOFF.md | internal | Branch handoff; delete once landed. |
| docs/WASM_BACKEND.md | keep | |
| docs/WASM_LINEAR_RUNTIME.md | internal | Plan. |
| docs/WORLD_INTEGER_DESCRIPTOR_HANDOFF.md | internal | Branch handoff; delete once landed. |
| docs/WORLD_MAIN_SOURCE_INTEGRATION_HANDOFF.md | internal | Branch handoff; delete once landed. |
| docs/WORLD_RESOURCE_DESCRIPTOR_HANDOFF.md | internal | Branch handoff; delete once landed. |
| docs/WORLD_SCHEMA_FORWARDPORT_PLAN.md | internal | Plan. |

## humankit

| File | Tag | Notes |
|---|---|---|
| BODY.md | merge | Layered pose model → `concepts/modelling/specialized-models.md` (humans); extension rules → `docs/guides/worker-behaviours.md`. |
| README.md | merge | 241 lines of API → `docs/guides/workers-and-jobs.md`; README keeps layout + tests. |
| TODO.md | internal | **Stale first line:** "next milestone is M5", but M5 landed (7e76fdb4c, a4fa89fee). |
| plans/M4-WORKERS.md | internal | Done plan. |
| plans/M5-WORKERS-IN-DOCUMENTS.md | internal | Done plan. |
| sim/README.md | merge | → `docs/guides/workers-and-jobs.md`. |
| sim/tests/README.md | internal | Demo/test recipe. |

## kinematicskit

| File | Tag | Notes |
|---|---|---|
| README.md | keep | Short, accurate. |
| native/THIRD_PARTY.md | keep | proxsuite notice. |
| plans/KINEMATICS.md | internal | Plan (K0–K6). |

## machinekit

| File | Tag | Notes |
|---|---|---|
| BOM_PLAN.md | internal | Plan. |
| ENCLOSURE_PLAN.md | internal | Plan. |
| END_EFFECTOR_PLAN.md | internal | Plan; EOAT conventions → `concepts/modelling/machine-components.md`. |
| END_EFFECTOR_TODO.md | internal | |
| GANTRY_PLAN.md | internal | 4,063-line plan. |
| GRIPPER_PLAN.md | internal | Plan. |
| LEVELING_FEET.md | keep | Shared component note; later generated reference. |
| METROLOGY_SOURCING.md | internal | Dated price snapshot. |
| PRECISION_ROTARY_PLAN.md | internal | Plan. |
| RAIL_MOUNTING.md | internal | Plan, nothing implemented. |
| README.md | merge | 548 lines: structural, transmission, assembly, robotics, screw-axis API → `concepts/modelling/machine-components.md`, `transmissions.md`, generated reference. "Sheet stock and cutting plans" section describes the picking station and ManufacturingKit, not MachineKit. |
| RESTRUCTURE_PLAN.md | internal | Plan (MK0–MK10). |
| STRAIN_WAVE_RESEARCH.md | internal | Research notes. |
| TODO.md | internal | |
| TRANSMISSION_PLAN.md | internal | Plan (X0–X10 done). |
| examples/bench-mill/AXIS_SIZING.md | internal | Sizing study. |
| examples/bench-mill/CAPACITY_STUDY.md | internal | Study; keeps an old project file as "compatibility shortcut". |
| examples/bench-mill/FEM_PLAN.md | internal | Plan. |
| examples/bench-mill/MOTOR_MOUNTS.md | internal | Screen. |
| examples/bench-mill/README.md | keep | Example entry; 482 lines, later → `docs/machines/bench-mill/`. |
| examples/bench-mill/REVIEW.md | internal | Code review. |
| examples/bench-mill/TOOLING_README.md | internal | Mold review (tooling phase 4). |
| examples/bench-mill/WORKHOLDING.md | internal | Design record. |
| examples/bench-mill/Y_SERVO.md | internal | Design record. |
| examples/bench-mill/fem/README.md | internal | FEM evidence log. |
| examples/box-mold/README.md | keep | Example entry. |
| examples/circular-spline-skiver/PLAN.md | internal | Plan. |
| examples/cnc-router/PLAN.md | internal | Done plan (C0–C5). |
| examples/cnc-router/README.md | keep | Example entry. |
| examples/cobot-arm/README.md | keep | Example entry. |
| examples/dh-gripper/README.md | keep | Supplier reference example. |
| examples/enclosure-kit/README.md | keep | Example entry. |
| examples/eoat/README.md | keep | Example entry. |
| examples/flexspline-hobber/ENCLOSURE.md | internal | Design record; later `docs/machines/flexspline-hobber/`. |
| examples/flexspline-hobber/HOB_GEOMETRY.md | internal | Design record. |
| examples/flexspline-hobber/MACHINE_DESIGN.md | internal | Design record; later `docs/machines/flexspline-hobber/`. |
| examples/flexspline-hobber/PARTS.md | internal | Assumed parts register. |
| examples/flexspline-hobber/PLAN.md | internal | 2,741-line plan. |
| examples/flexspline-hobber/README.md | keep | Example entry. |
| examples/flexspline-hobber/SOCKET_FIXTURE.md | internal | Coupon record. |
| examples/foam-cutter/PLAN.md | internal | Plan. No README for the example. |
| examples/gantry-picker/README.md | keep | Example entry; the tutorial covers it. |
| examples/lathe/PLAN.md | internal | Plan. |
| examples/leadshine-servos/README.md | keep | Catalogue example. |
| examples/linear-rail-reference/FAMILIES.md | keep | Catalogue families. |
| examples/linear-rail-reference/README.md | keep | Supplier reference example. |
| examples/machine-tending/PLAN.md | internal | Plan (MT0–MT14). |
| examples/machine-tending/README.md | keep | Example entry. **No H1 title**; 692 lines open mid-topic on the electric wrist. |
| examples/machine-tending/REVIEW_FIXES.md | delete | Self-described historical MT1–MT6 checkpoints, "not current"; superseded by PLAN.md. |
| examples/mobile-base/PLAN.md | internal | Plan. |
| examples/mobile-base/README.md | keep | Example entry. |
| examples/overhead-arm/README.md | keep | Example entry. |
| examples/panel-tooling/README.md | keep | Example entry. |
| examples/picking-station/README.md | keep | Example entry. |
| examples/press-brake/README.md | keep | Example entry. |
| examples/printer/PLAN.md | internal | Plan ledger. No README for the example. |
| examples/reference-arms/ARM-DESIGN.md | internal | Design baseline. |
| examples/reference-arms/COMMERCIAL-REFERENCE-VALIDATION.md | internal | Evidence. |
| examples/reference-arms/PASSIVE-DYNAMICS.md | internal | Investigation. |
| examples/reference-arms/PLAN.md | internal | Plan. |
| examples/reference-arms/README.md | keep | Example entry; 642 lines, mostly status. |
| examples/reference-arms/REVIEW.md | internal | Review. |
| examples/robot-arm/README.md | keep | Example entry. |
| examples/robot-screwdriver/PLAN.md | internal | 4,168-line plan (SD0–SD15 done). |
| examples/robot-screwdriver/README.md | keep | Example entry. |
| examples/robot-welder/HANDOFF-W4-W6.md | delete | Handoff for W4–W6; W6 landed (8a209d9ff "complete W6 validation and phase-one landing record"). PLAN.md is the record. |
| examples/robot-welder/PLAN.md | internal | Plan. |
| examples/robot-welder/README.md | keep | Example entry. |
| examples/sheet-router/PLAN.md | internal | Plan (SR0–SR24). |
| examples/sheet-router/SOURCING.md | merge | Approved catalogue basis → `docs/machines/sheet-router/bom.md` (D8). |
| examples/sheet-router/atc/README.md | internal | SR7 evidence. |
| examples/sheet-router/dxf/README.md | merge | `Dxf.read` usage → `docs/guides/import-dxf.md`; assertion counts stay. |
| examples/sheet-router/fabrication/DIY_ROUTE.md | merge | → `docs/machines/sheet-router/fabrication.md` (D8). |
| examples/sheet-router/fabrication/README.md | merge | → `docs/machines/sheet-router/fabrication.md`; "draft, not released". |
| examples/sheet-router/fabrication/WELD_QUALIFICATION.md | merge | → `docs/machines/sheet-router/fabrication.md` (weld qualification). |
| examples/sheet-router/fem/README.md | internal | SR5 evidence log. |
| examples/sheet-router/sheet-scale/README.md | internal | SR9 evidence. |
| examples/sheet-router/vacuum/README.md | internal | SR8 screen. |
| examples/spindle-drive-study/README.md | internal | Study. |
| examples/spindle-drive-study/sources/leadshine-curves/README.md | keep | Source provenance. |
| examples/supplier-fasteners/SFU20_AXIS.md | keep | Component example. |
| examples/tool-service/FAIRINO-ADAPTERS.md | keep | Component note. |
| examples/tool-service/README.md | keep | Example entry. |
| examples/zeroerr-arm/ARM-BENCH-PLAN.md | internal | Plan. Cites MODULE-MASS-AUDIT as "the current register". |
| examples/zeroerr-arm/ARM-OPTIONS.md | delete | Banner: "Historical experiment / superseded load interpretation"; superseded by REFERENCE-RECONCILIATION.md, listed in EXPERIMENTS-ARCHIVED.md. |
| examples/zeroerr-arm/CLEARANCE.md | internal | Review of an archived experiment. |
| examples/zeroerr-arm/EXPERIMENTS-ARCHIVED.md | internal | Archive index. |
| examples/zeroerr-arm/FROZEN-REFERENCE-STATUS.md | internal | Status. |
| examples/zeroerr-arm/J2-BENCH.md | internal | Bench package. |
| examples/zeroerr-arm/MODULE-MASS-AUDIT.md | internal | Banner says superseded, yet ARM-BENCH-PLAN cites it as current: reconcile, then delete or drop the banner. |
| examples/zeroerr-arm/ORIGINAL-EVALUATION.md | internal | Evaluation. |
| examples/zeroerr-arm/ORIGINAL-REFINEMENT.md | internal | Evaluation. |
| examples/zeroerr-arm/README.md | keep | Example entry; 565 lines, mostly status. |
| examples/zeroerr-arm/REFERENCE-BUILD.md | internal | Build package; later `docs/machines/`. |
| examples/zeroerr-arm/REFERENCE-CONTROLS.md | internal | Evidence. |
| examples/zeroerr-arm/REFERENCE-RECONCILIATION.md | internal | Current authority for the arm. |
| haxe/src/machinekit/design/README.md | merge | `DesignInterfaces` checks → `concepts/modelling/machine-components.md`. |

## manufacturingkit

| File | Tag | Notes |
|---|---|---|
| README.md | merge | 428 lines: sheet plans → `docs/guides/sheet-cutting-plans.md`; casting tooling → `docs/guides/casting-tooling.md`; idea → `concepts/process/manufacturing-outputs.md`. |
| TOOLING_PLAN.md | internal | Plan (phases 1–9). |

## motionkit

| File | Tag | Notes |
|---|---|---|
| IMPLEMENTATION_PLAN.md | internal | Done plan (P1–P11). |
| README.md | merge | API → `concepts/motion/paths-and-trajectories.md`, `execution-plans.md`, `machine-axes.md`; "what validated means" → `execution-plans.md`. |
| native/THIRD_PARTY.md | keep | Ruckig/TOPP-RA/OPW notices. |
| plans/CONTRACTS.md | internal | Contracts C1–C5. |
| plans/EAIK_SPIKE_RESULTS.md | internal | Spike result. |
| plans/HARDENING.md | internal | Plan. |
| plans/LANE_A_VIRTUAL_DEVICE.md | internal | Plan. |
| plans/LANE_B_ARM_PROCESS.md | internal | Plan. |
| plans/LANE_C_PLANNING.md | internal | Plan. |
| plans/LANE_D_REDUNDANCY_SERVO.md | internal | Stub; D1–D3 moved to KINEMATICS.md K3. |
| plans/ONLINE_MOTION.md | internal | Plan (J0–J7). |
| plans/PROCESS_PATH_PLANNING.md | internal | 3,437-line plan. |
| plans/README.md | internal | Plan index. |

## processkit

| File | Tag | Notes |
|---|---|---|
| README.md | merge | Package map → `concepts/process/process-and-robots.md`; screwdriver, Klipper, Moonraker, enclosure guard → `docs/guides/`. |

## projectkit

| File | Tag | Notes |
|---|---|---|
| README.md | merge | → `docs/reference/file-formats/mtrg.md` + `concepts/foundations/data-and-runtime.md`. **Stale:** says MTRG version 29; `SceneArtifact.VERSION` is 41. Still calls `AssemblyRecord` "legacy" alongside `AssemblyDefinition`. |
| tests/decoding/README.md | internal | Benchmark. |

## robotkit

| File | Tag | Notes |
|---|---|---|
| ARCHITECTURE.md | merge | 1,889 lines; object model and frame conventions → `concepts/execution/robot-runtime.md`, `concepts/foundations/conventions.md`; the rest stays as a contributor document (internal). |
| CONSTRUCTION_ROADMAP.md | internal | Done roadmap (M0–M12). |
| FOLLOWUP_PLAN.md | internal | Follow-ups to the roadmap. |
| MOTION_ARCHITECTURE_GAP_MAP.md | delete | Self-described "historical snapshot from before Lane A"; superseded by `runtime/DEVICE_PROTOCOL.md` and the completed IMPLEMENTATION_PLAN. Drop the reference in `motionkit/IMPLEMENTATION_PLAN.md`. |
| README.md | merge | 557 lines; package table stays; control snapshots/streams → `concepts/execution/robot-runtime.md`. |
| RESTRUCTURE_PLAN.md | internal | Plan (R0–R6 done). |
| TODO.md | internal | |
| deployment/bench-nucleo-g474re/README.md | merge | → `docs/guides/bench-board.md` once hardware exists. |
| device_protocol/README.md | keep | Short crate README. |
| device_protocol/boards/nucleo-g474re/README.md | keep | Board adapter README. |
| haxe/README.md | keep | 3-line package note; no H1 title. |
| plans/HUMANOID.md | internal | Plan. |
| plans/MIXED_SCENE.md | internal | Investigation. |
| plans/PERCEPTION.md | internal | **Stale status:** "plan, not started", but `robotkit/autonomy/.../perception/` exists. |
| plans/SENSORKIT_CAMERA.md | internal | Spike decision. |
| plans/TOOL_CHANGE.md | internal | Plan. |
| robotd/README.md | keep | Executable README. |
| runtime/DEVICE_PROTOCOL.md | merge | → `docs/reference/protocols/rkd6.md`; boundary → `concepts/execution/controller-boundary.md`. |
| runtime/ELECTRONIC_GEAR.md | merge | → `docs/guides/electronic-gearing.md`. |
| schema/README.md | keep | Short; overlaps DEVICE_PROTOCOL's revision rules. |
| tests/fixtures/humanoid/README.md | keep | Fixture licence/provenance. |
| worldd/README.md | keep | Short, accurate. |

## scenekit

| File | Tag | Notes |
|---|---|---|
| FOLLOW_UPS.md | internal | TODO under another name. |
| README.md | merge | Scene identity → `concepts/foundations/identity.md`; libraries → `concepts/platform/rendering.md`. |
| scene/README.md | merge | Titled "NativeKit Scene contracts" (pre-split name). World conventions duplicate `concepts/foundations/conventions.md`; the C ABI rules → `docs/reference/scenekit-abi.md`. |
| scene_interaction/haxe/uikit/README.md | keep | Adapter note. |
| scene_render/shaders/README.md | keep | Shader regeneration, contributor-facing. |

## sensorkit

| File | Tag | Notes |
|---|---|---|
| README.md | merge | → `concepts/foundations/truth-and-observation.md`; wire part → `docs/reference/protocols/sensor-wire.md`. |
| reference/README.md | internal | Notes on an ignored local checkout. |
| sensor_io/generated/sensor_wire.md | keep | Generated wire reference; link from `docs/reference/`. |

## simkit

| File | Tag | Notes |
|---|---|---|
| README.md | keep | Short; still describes NativeKit and SceneKit as "sibling checkouts" (NativeKit now lives in `haxeon/vendor/nativekit`). |
| TODO.md | internal | |
| plans/FIDELITY.md | internal | Plan; concept page `concepts/execution/fidelity.md` exists. |
| sim_core/README.md | merge | Titled "NativeKit simulation core". Host modes and ownership → `concepts/execution/simulation-worlds.md`. |
| sim_mujoco/README.md | merge | Backend behaviour → `concepts/execution/simulation-worlds.md`; build flags stay. |

## stockkit

| File | Tag | Notes |
|---|---|---|
| HOBBING_ERRORS.md | internal | HB11 engineering record. |
| HOBBING_FORCES.md | internal | HB10 engineering record. |
| README.md | merge | Tri-dexel idea → `concepts/process/stock.md`; API → guides. Heading "phase 6 under way" disagrees with TODO.md ("phase 6 done apart from manifold contouring"). |
| TODO.md | internal | |
| docs/DESIGN.md | merge | Design rationale (CAMotics/FreeCAD comparison, OpenVDB decision) → `concepts/process/stock.md`. |
| docs/PLAN.md | internal | Plan. |
| docs/REFERENCES.md | internal | Research notes. |
| tests/components/README.md | internal | Test notes. |

## toolpathkit

| File | Tag | Notes |
|---|---|---|
| README.md | keep | Short, accurate. |
| docs/ADR-001-toolpath-ir.md | internal | ADR; decisions already in `concepts/process/toolpaths.md`. |
| docs/NATIVE_PROGRAM_PLAN.md | internal | Plan (unscheduled). |
| docs/PLAN.md | internal | T0–T7 complete; still cited by NATIVE_PROGRAM_PLAN and the README. Delete once those references go. |
| motion/README.md | keep | Short adapter README. |

## tools

| File | Tag | Notes |
|---|---|---|
| suppliers/README.md | internal | 585-line importer notes. |
| suppliers/zppcnc-actuation-review.md | internal | Supplier evidence review. |
| wire/DESIGN.md | merge | → `docs/reference/wire-idl.md`. |
| wire/PROFILE.md | merge | `.wire.idl` grammar → `docs/reference/wire-idl.md`. |

## trajectorykit

| File | Tag | Notes |
|---|---|---|
| README.md | keep | Short; idea already in `concepts/motion/execution-plans.md`. |

## visionkit

| File | Tag | Notes |
|---|---|---|
| PLAN.md | internal | **Stale status:** "plan, not started", while the README documents an implemented camera/PnP/marker kit. |
| README.md | keep | |
| TODO.md | internal | |
| native/THIRD_PARTY.md | keep | OpenCV notice. Names a host-specific path (`/home/joao/dev/materia-deps/...`). |

## Stability

Nothing in Materia is stable: no kit has a release, a frozen API or a
compatibility promise, and the no-legacy rule means formats change without
readers for old versions. **unstable** means tested, used by the editor or
other kits, and changing weekly. **experimental** means narrow, recent,
lightly used, or with a planned redesign. Commit counts are
`git log --since=2026-09-01 --oneline -- KIT | wc -l` (Haxeon packages counted
in the Haxeon repo). "Workspace" means a test project in
`materia.workspace.json`.

| Kit | Label | Evidence |
|---|---|---|
| animkit | experimental | 24 commits; implementation moved to Haxeon animation on 2026-10-12, adapter left here; workspace tests. |
| automationkit | experimental | 25 commits; one facility used, hard-coded in the app; workspace tests. |
| bimkit | experimental | 14 commits; "deliberately narrow" (straight walls only); no workspace test project. |
| cadkit | unstable | 320 commits; native ctest + Haxe smoke; core of the editor; ABI additive but Haxe layer churning (TN, sketcher). |
| camkit | experimental | 51 commits; workspace tests; 2.5D only plus hobbing. |
| cnckit | unstable | 43 commits; workspace tests; used by the editor's CNC players. |
| collisionkit | experimental | 37 commits; CL9 shape vocabulary redesign planned; coal fork with unpushed fixes; tests not in the workspace. |
| gearkit | experimental | 6 commits; tooth-design plan not started; no README. |
| humankit | experimental | 75 commits; workspace + MuJoCo tests; open IK and motion bugs in TODO. |
| kinematicskit | unstable | 52 commits; used by MotionKit, RobotKit, CadKit; tests not in the workspace. |
| machinekit | unstable | 1,185 commits; 4-shard workspace suite; every example machine depends on it; MK6/8/9 restructure pending. |
| manufacturingkit | experimental | 49 commits; casting tooling mid-plan (phases 6–9 open), codecs at version 2. |
| motionkit | unstable | 583 commits; workspace + planner tests; OM-R1 jerk timing in progress. |
| processkit | unstable | 178 commits; workspace tests; split out of RobotKit in R0–R6. |
| projectkit | unstable | 179 commits; MTRG went 29 → 41 since its README was written (97 commits to SceneArtifact.hx alone). |
| robotkit | unstable | 798 commits; core, integration and MuJoCo suites; RKD6 wire revision 16, frozen only at first hardware. |
| scenekit | unstable | 85 commits; native tests; the editor's renderer. |
| sensorkit | experimental | 25 commits; README untouched since 2026-09-25; editor does not use it. |
| simkit | unstable | 142 commits; native ctest; MuJoCo backend used by every Play; fidelity levels planned. |
| stockkit | experimental | 55 commits; workspace tests; phase 7 (multi-axis) and tiling open. |
| toolpathkit | experimental | 79 commits; workspace tests; NATIVE_PROGRAM_PLAN NP-D2 would change tool identity in the format. |
| trajectorykit | experimental | 13 commits; new split from MotionKit/RobotKit; tested through RobotKit native. |
| visionkit | experimental | 11 commits; only RobotKit autonomy uses it; plan status stale. |
| haxeon-platform | unstable | 14 commits; every native app depends on it. |
| haxeon-ui | unstable | 101 commits; 71 test files; the editor UI; moved from Materia's uikit recently. |
| haxeon-gpu | unstable | 5 commits; under UI and SceneKit. |
| haxeon-editor | experimental | 2 commits since the move; workspace tests. |
| haxeon-animation | experimental | 1 commit (the move from AnimKit). |
| haxeon-audio | experimental | 21 commits; not used by Materia. |
| haxeon-filesystem | experimental | 1 commit. |
| haxeon-credentials | experimental | 1 commit. |

D0 also asks for these labels in `materia.workspace.json`; that edit is
not part of this file.

## Site map

The target `docs/` tree, from DOCS_PLAN. Existing sections are marked.

```
docs/
  README.md        exists: site landing
  concepts/        exists: 35 pages + index + glossary, all status: draft
  tutorials/       exists: index + gantry-picker (draft)
  guides/          missing: build-from-source, web-build, cad-modeling, cam-2.5d,
                   gcode, printer-gcode, hobbing-cam, import-dxf, export-ifc,
                   sheet-cutting-plans, casting-tooling, workers-and-jobs,
                   worker-behaviours, project-ui-extensions, electronic-gearing,
                   gear-inspection, bench-board
  manual/          missing: editor user manual, from app/README.md
  reference/       missing; docs/api/ (untracked) is the start of the generated part.
                   Hand-written: file-formats/mtrg, protocols/rkd6,
                   protocols/sensor-wire, wire-idl, cadkit-abi, scenekit-abi,
                   project-ui-extension; plus links to Haxeon's CLI/language docs
  contributing/    missing: build, tests, worktrees, docs-site, performance-gates,
                   third-party notices, Haxeon notes (CONTRIBUTING.md links here)
  machines/        missing: sheet-router first (D8), then bench-mill,
                   flexspline-hobber, zeroerr-arm
  products/        missing (later)
  OPEN_CORE.md     exists
  DOCS_PLAN.md     exists
  INVENTORY.md     this file
```

Also under `docs/` but not content: `site/` (Starlight), `figures/` (baked
figure program), `runtime/`.
