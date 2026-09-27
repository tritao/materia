# Motion stack plans

| File | What it is |
| --- | --- |
| `../IMPLEMENTATION_PLAN.md` | Completed foundation (P1–P11): native polynomial trajectories, validation, Ruckig, runtime execution sessions, native HOLD/RESUME/ABORT, plans over serial and robotd, model transmissions. Its decisions D1–D4 and ground rules still apply. |
| `CONTRACTS.md` | **Start here.** Shared contracts C1–C5 used by more than one lane, cross-lane rules, and §P0: the one session that lands the shared types before the lanes start. |
| `LANE_A_VIRTUAL_DEVICE.md` | RKD6 scheduled device protocol, virtual MCU, clock sync, device-side stop, step generation, SimKit loop, transmissions end to end, then retiring RKD5 in favour of a minimal RKD6 profile (A9). |
| `LANE_B_ARM_PROCESS.md` | Arm toolpaths as validated plans, path-synchronized process events, `ManipulatorMotion`, wall finishing and excavator migration, ProcessKit. |
| `LANE_C_PLANNING.md` | Native path trajectories and TOPP-RA timing, corner blending, OPW IK, configuration selection, CncKit, virtual CNC end to end. |
| `LANE_D_REDUNDANCY_SERVO.md` | **Stub, not scheduled.** Native mink-shaped QP differential IK (OSQP), 7-axis redundancy, live Cartesian servoing through the execution session, collision-avoidance limits, coordinated external axes. Uses mink as the design reference and an optional test oracle. |

**Order:** §P0, merged to `main`, then Lanes A, B and C in parallel, each in
its own worktree.

**Cross-lane dependencies:**
- Lane A's A8 needs Lane B's B2.
- Lane B's B8 needs Lane C's C2.
- Lane C's C7 uses Lane A's A6 when it is available.

Everything is virtual: no hardware is required.
