# Executed J6 rerun — 2026-10-26

Frozen-frame baseline is complete: both beds, 100/50 mm meshes, all nine full-Z
poses, 40 modes per pose. All 186 producer histories pass manifest/output/provider
hash validation; 372 pose/mesh responses use the executed motion, including the
full feed-resume histories. No frozen CAD changed. Cycle times are excluded.

| Frozen reference | 2600 bed | 3150 bed |
|---|---:|---:|
| Worst fine-mesh static X, N/µm | 2.165 | 2.109 |
| Worst fine-mesh static Y, N/µm | 2.051 | 2.044 |
| Nominal braking-following, mm | 0.05024 | 0.06866 |
| Observed uncertainty braking-following, mm | **0.07870** | **0.09967** |
| Nominal geometric contour deviation, mm | 0.01513 | 0.02200 |
| Nominal physical-rest residual, mm | 0.00466 | 0.00532 |

Both static targets pass. Both braking-following screens fail the requested
0.05 mm target (and 0.04 mm reserve). Geometric contour and rest residual remain
separate quantities; the nominal figures are not uncertainty bounds or a new
SR-D3 release. No producer structural-response numbers were transplanted.

The braking uncertainty screen evaluates all histories at 17 frequency scales
from 0.85–1.15 and damping ratios 0.5%, 1%, 3%. It then refines two screened
governing histories per bed at 129 scales and seven damping ratios from 0.5–3%.
These are sampled searches, not continuous uncertainty qualification or a full
129×7 sweep on every history. Modal history starts at the rapid lead-in. Each
screened peak is rechecked at the export timestep after the 1 ms coarse response.
Nominal values use the computed frequencies and assumed 1% damping.

Governing screened braking poses: X800/Y0/Z200 on the short bed and
X800/Y3150/Z200 on the long bed, on the 50 mm mesh. The short-bed governing
history is phase250/p3 hold/resume; long-bed is phase500/p5 hold/resume. Their
coupled frame-X/gantry-Y modes lie near 27 Hz. The frame-X mode stores energy in
frame/feet/braces; the gantry-Y roll mode involves beam bending, Y guides and the
Y drive. At short-bed centre, the latter has approximately 34% beam and 32% Y-drive
strain energy; this supports ranking the motor mount/drive ahead of local plates.

All 36 structural cases ran serially through `heavy-run --slots 1` with a
3,000,000,000-byte scope cap, an 8 GiB available-RAM gate, an immediate pre-CCX RAM
check and an 8 GB root-space floor. Maximum measured scope peak was 919,220,224
bytes. No completed case retains bulk solver output. The motion landing suite has
priority; structural options wait for its shared slot.

[Compact results and verified archive inventory](rack-results/j6-frozen-20261026.json)
retain source/packet/deck hashes, per-case stiffness, first-mode energy shares,
response provenance and resource evidence. The hash-verified compact archive is
on root; producer histories/providers remain at the handoff location. No DATA1
writes. The existing 15 preparation tests pass.

Next: drive/motor-mount and end-anchor screens at the two governing poses,
including an explicitly illustrative gross-section mount-mass sensitivity, then
shaping and reduced-limit bounds. Those policy bounds are offline transformed
loads, not newly executed J6 exports. Real mount geometry, full-length/X-host
qualification, native clearance fixes, the mount-required API migration,
higher-mode convergence and measured-mode/damping commissioning remain held.
