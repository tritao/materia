# Robot arm design plan

Current priority: [standalone arm design and reference validation](ARM-DESIGN.md).
The cell/tending work below is historical and deferred. A complete commercial
controller oracle, hardware capability and physical qualification are not implied
by the earlier completed software checklist.

## Active frozen-reference steps

- [x] 1. Freeze original supplier geometry, archive modification experiments, replace shell masses with whole-module lumps and reconcile 5 kg static/duty checks. See [reconciliation](../zeroerr-arm/REFERENCE-RECONCILIATION.md).
- [x] 2. Final [supplier-confirmation request](../zeroerr-arm/SUPPLIER-CONFIRMATION.txt); ready-to-send text saved, not sent.
- [ ] 3. STEP-derived BOM, assembly/cable/EtherCAT wiring and rough cost.
- [ ] 4. Drive profile, timed simulation/control integration and EtherCAT/eRob stub; unknown commissioned limits remain explicit.
- [ ] 5. One 110T J2 bench package, measured load, before six-module purchase.
- [ ] 6. Independent FR5 manufacturer kinematics validation beside UR5e.

No geometry modifications. Treat the supplier 5 kg rating as trusted. CNC tending
and the old 23 transfers are deferred; eventual cell design uses shared commercial
and original-arm workspace.

## Historical comparison and tending work

Implementation and software/design review completed. Acceptance evidence and
negative engineering findings are consolidated in [REVIEW.md](REVIEW.md).
Hardware, manufacturing and complete cell release remain unqualified.

## Acceptance checklist

- [x] Reproducible official UR and FAIRINO description/mesh acquisition, pinned provenance and licence records; generated vendor assets excluded from Git.
- [x] UR5e and FR5 articulated scene exports with correct scale, axes, limits, base and flange frames.
- [x] Existing FK/IK integration checked against independent URDF FK, multiple branches and joint limits; joint jogging and Cartesian trajectories.
- [x] Shared comparison scene with actual-scale robots, common tool/task frames, switchable mounting and overview/close-up previews.
- [x] Extend the working importer to the available UR and FAIRINO ranges; report model-specific gaps honestly.
- [x] Separate straight-link ZeroErr variant preserving 425/401 mm axis spacing and supplier interfaces, with structural adapters, fastening/cable access and separate covers.
- [x] Valid connected solids, mass/inertia, stiffness/connection-load estimates and comparison against tapered baseline.
- [x] Updated workspace/tending load reports; unresolved shoulder sizing clearly retained.
- [x] Diagnostic and interpolated tending clearance review, with collisions and failed queries recorded and long work checkpointed.
- [x] Actual fixture/TCP registration required for real-cell feasibility; no invented cell certificate.
- [x] Draft prototype BOM, interface drawings, missing supplier data and single-joint bench procedure updated.
- [x] Relevant commits, runnable commands, artifact paths and final completion audit.

No purchase, supplier messages or physical tests are authorized by this software/design task.
Existing unrelated edits remain untouched. Visual/kinematic simulation does not qualify hardware dynamics.

## Final findings and evidence

See [REVIEW.md](REVIEW.md) for each requirement's evidence and scope. Generation,
application and verification commands are in [README.md](README.md) and the
[ZeroErr README](../zeroerr-arm/README.md). Run `audit-deliverables.py` after the
documented native checks and application integration test; it writes the current
`build/completion-audit.json` integrity and coverage record.

The study retains these negative findings:

- No reviewed drive is selected for the full shoulder torque/speed target. The
  registered actual-tool configuration screen also exceeds the current shoulder
  candidate under its conservative assumptions.
- None of the three candidate layouts establishes a complete tending cycle:
  4/23, 10/23 and 18/23 transfers have continuous checks for our restricted arm, UR5e and
  FR5 respectively. Failed endpoints/routes and search limits remain recorded.
- Folded self-collisions and distal-cover contact limit the explored envelope.
  The follow-up interference review reduces cover overhang and adds J3/J4
  simulation restrictions; all 54 retained observed poses are clear. The five
  extreme collision witnesses remain recorded outside the window. All nine
  candidate bases were planned again, with a 4/23 best restricted result versus
  the previous unrestricted 5/23; shoulder demand reaches 166.01 Nm on the
  conservative registered-configuration screen.
- Mechanical fits, cover retention, cables, physical commissioning and hardware
  qualification remain open; prototype drawings/BOM/bench request remain drafts.
- Two malformed FAIRINO long-reach URDFs, unavailable UR STEP access, unverified
  wider STEP/flange correspondence and unresolved FAIRINO redistribution terms
  are reported source gaps. Vendor assets remain generated and excluded from Git.

These findings are outcomes of the review, not positive hardware or cell release
claims. Further mechanical/layout/procurement/physical work is required before
real-world testing beyond the documented, separately reviewed single-joint bench.
