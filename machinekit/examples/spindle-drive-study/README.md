# Spindle drive requirements: BT30 mill and flexspline hobber

Study date: 2026-10-19. The working choice is a spindle-duty motor with a vector
drive for each cutting spindle, and a position-controlled torque motor for the
hobber work table. The drive must expose the feedback and control functions
required below. This chooses an architecture and provisional performance targets;
it does not qualify a motor, drive, existing CAD envelope or machining accuracy.

The mill needs a useful continuous torque range. The hobber needs a measured-angle
electronic gearbox. A general-purpose VFD with an rpm command alone cannot provide
that synchronization, while using an AC servo alone does not establish it either.

![Calculated mill torque envelope and assumed hobber control-error bounds](spindle-drive-study.png)

## Existing design basis and new assumptions

The [BT30 packaging study](../bench-mill/README.md#bt30-spindle-packaging-study)
specifies a 2.2 kW / 8000 rpm concept and 1:1 belt drive. Its compact motor is
assumed geometry. The older ER20 example uses a different, assumed 1.1 kW motor;
its bearing-block program requests 8000 rpm. These are separate configurations.

This study interprets the BT30 target as **2.2 kW useful spindle output**. That is
a new explicit interpretation: the current 2.2 kW motor would provide only
2.09 kW after the assumed 5% belt loss. The new low-speed envelope, minimum speed
and efficiency are provisional requirements, not established cutting demands.
They must eventually be checked against tool diameter, material, engagement,
feed, spindle thermal limits and the actual electrical supply.

The [hobber reference](../flexspline-hobber/PLAN.md#reference-numbers-assumed-until-checked-against-a-datasheet-or-a-measurement)
is m=0.3, 200 teeth, 60 mm pitch diameter, a single-start 32 mm hob with 12 gashes,
3000 rpm reference cutting speed, and an 8000 rpm maximum. It already calls for
a direct-drive spindle around 1.5 kW, a direct-drive C table and load-side ring
encoder. The 1 µm control allocation below is new. It is only one contribution
to tooth error; it is not a complete gear tolerance or an ISO grade claim.

Inputs and their provenance are in [requirements.json](requirements.json).
Numerical results are in [results.json](results.json).

## Mill: provisional continuous output envelope

Use 500–8000 rpm, constant torque to an assumed 3000 rpm base, then constant
power. This deliberately asks for more low-speed torque than a motor whose
2.2 kW rating occurs only at 8000 rpm.

| Spindle speed | Continuous output torque | Useful output power |
| --- | --- | --- |
| 500 rpm | 7.00 Nm | 0.367 kW |
| 1000 rpm | 7.00 Nm | 0.733 kW |
| 1500 rpm | 7.00 Nm | 1.100 kW |
| 3000 rpm | 7.00 Nm | 2.200 kW |
| 6000 rpm | 3.50 Nm | 2.200 kW |
| 8000 rpm | 2.63 Nm | 2.200 kW |

Calculations: `P = T·2πn/60`; below base speed, `T = 2200/ω(3000)`.
This does **not** ask for 2.2 kW at 500 rpm, which would require 42 Nm.
With 95% mechanical transmission efficiency, the motor must deliver at least
2.316 kW at the high-speed operating point. Investigate a **3 kW class spindle-duty
motor and compatible vector drive** as the first sourcing target. A 3 kW label
alone is insufficient: its continuous curve must cover the entire envelope,
including any field-weakening region and low-speed thermal derating.

Retain 1:1 only if a supplier motor genuinely meets the 8000 rpm requirement.
An ordinary 3000 rpm induction motor must not be assumed mechanically suitable
for 8000 rpm. A belt ratio, independently cooled motor or two ranges may be needed;
each changes torque, shaft loads, balance and the current head packaging.

Provisional ordinary-milling speed regulation target: ±1% of commanded speed at
steady load. This is a chip-load/stability target, **not a dimensional tolerance**.
Measure load-step speed droop, recovery, temperature and low-speed operation;
we have not assigned a recovery-time limit without process data. Reserve a
spindle-side encoder interface, index and CNC integration for orientation and
future rigid tapping. Rigid tapping remains unqualified until reversal, Z/spindle
phase error and recovery are demonstrated with the actual controller.

## Screening the modeled Leadshine alternatives

The following uses only published rated torque at rated speed, a belt speed
increase to 8000 rpm, and the assumed 95% efficiency. It does not extend rated
torque to maximum speed or use transient peak torque as continuous cutting torque.

| Motor, no brake | Motor rated speed | Speed increase | Output torque at 8000 rpm | Motor mass | Result at this point |
| --- | --- | --- | --- | --- | --- |
| ELM2L-1500LA100FT-H | 3000 rpm | 2.667× | 1.75 Nm | 5.8 kg | Below mill's 2.63 Nm requirement |
| ELM2M-2000LB130F-H | 2500 rpm | 3.200× | 2.29 Nm | 8.3 kg | Below mill's 2.63 Nm requirement |
| ELM2M-7500LD180FT-H | 1500 rpm | 5.333× | 8.55 Nm | 32.3 kg | Enough at this point; poor compact-head candidate |

At 500 spindle rpm these fixed ratios put the motors at 188, 156 and 94 rpm
respectively. Continuous low-speed torque/cooling needs separate verification.
The 7.5 kW motor's 180 mm frame cannot use the assumed Ø90 mm motor allocation
without a new head design and payload assessment. No row is a qualified mill
motor. None reaches 8000 rpm directly, so none satisfies the hobber's present
direct-drive maximum-speed requirement either. Belt-driving the hob would be
a deliberate change to its direct-drive architecture and error budget.

Source: [Leadshine ELM1/ELM2 datasheet, pp. 9–16](https://www.leadshine.com/upfiles/downloads/006a40c553d0c6fc60a1221d6a764222_1675822261873.pdf).
Other catalog rows have not been exhaustively screened here.

## Hobber: power and synchronization requirements

At 3000 rpm the table turns at 15 rpm, and at 8000 rpm it turns at 40 rpm.
For the single-start, 200-tooth spur reference:

`C_target(t) = B_measured(t) / 200 + C_programmed(t)`.

Use unwrapped, timestamped angles and the correct signed ratio. Helical and hob
shift compensation require the extra terms already specified in HB-D2. Measure
hob rotation at the actual spindle and work rotation at the table; motor-only
feedback cannot observe belt slip, compliance or downstream transmission error.
Motor commutation/drive feedback may still be separately required.

Provisional output target: 1.5 kW at 8000 rpm, equivalent to 1.79 Nm. A sensitivity
calculation for 20/50/100 N **tangential** force at the 16 mm hob radius gives
0.32/0.80/1.60 Nm. At 3000 rpm that is 0.101/0.251/0.503 kW, and at 8000 rpm it is
0.268/0.670/1.340 kW. These force values are assumed scenarios, not a conversion
of the plan's unspecified cutting-force direction into motor load. The 100 N
case leaves only about 12% output-power headroom at 8000 rpm before acceleration
and additional loads. Keep 1.5 kW provisional until chip-load force prediction,
arbor/bearing loss, inertia and acceleration duty have been checked.

Allocate **≤1 µm peak absolute tangential control error at r=30 mm**, equivalent
to 6.88 arcseconds of work-table error. The worst-case pitch-position difference
from this contribution alone can be 2 µm between two samples. This is distinct
from single-pitch, cumulative-pitch, profile and helix metrics. The existing
plan's class 5–6 estimates below module 0.5 remain extrapolations requiring a
separate metrology/tolerance decision; this study does not certify those limits.

| Allocated source / provisional bound | Error at 3000 hob rpm | Error at 8000 hob rpm |
| --- | --- | --- |
| Table encoder installed accuracy ±2″ | 0.291 µm | 0.291 µm |
| Table quantization, half count at 2²³ counts/rev | 0.011 µm | 0.011 µm |
| Hob encoder installed accuracy ±60″, referred through 1:200 | 0.044 µm | 0.044 µm |
| Hob quantization, half count at 16384 counts/rev, referred | 0.029 µm | 0.029 µm |
| Residual timestamp/clock error ≤1 µs | 0.047 µm | 0.126 µm |
| Unmodeled 1000 rpm/s hob acceleration over 250 µs prediction | 0.0005 µm | 0.0005 µm |
| C-axis following residual ≤2″ | 0.291 µm | 0.291 µm |
| Unmeasured torsion ≤1″ referred to work | 0.145 µm | 0.145 µm |
| **Sum of absolute bounds** | **0.858 µm** | **0.937 µm** |
| Remaining control allocation | 0.142 µm | 0.063 µm |

These are **requirements to prove**, not observed performance. Bounds are added
arithmetically; no statistical independence is claimed. Encoder accuracy must
include mounting/eccentricity and relevant interpolation errors; resolution alone
does not specify it. The accuracy and half-count terms are separate here; if a
supplier's system-accuracy specification already includes quantization, reconcile
that definition before combining it. Define following residual consistently to
avoid counting metrology error twice in later measured budgets.

At the reference speed, 125/250/1000 µs of **uncompensated** delay produces
5.89/11.78/47.12 µm of error. At maximum speed the same delays produce
15.71/31.42/125.66 µm. A faster nominal loop alone does not meet the budget.
Time-align measurements and predict across measured constant delay; the 1 µs
bound applies to the remaining timestamp/clock uncertainty, not to all network
transport time. Constant-velocity prediction has a residual `0.5·α·τ²`; the
small computed value assumes the stated acceleration bound and is not a model
of gash-frequency torque ripple, velocity-estimation noise or torsional modes.

Start with 4–8 kHz EGB/table loops, hardware timestamping or equivalently bounded
latching, a common timebase, and maximum prediction horizon 250 µs. At 8000 rpm,
12 gashes produce 1600 Hz tooth-passing excitation: 4/8 kHz loops offer only
2.5/5 samples per gash. Those rates do not establish disturbance rejection.
Measure speed ripple and true table/hob phase error under cutting and confirm
adequate bandwidth, filtering and structural response. Extra velocity-estimation
or filtering delay must fit the same timing model. The small remaining 0.063 µm
budget at maximum speed is a reason to measure first, rather than claim accuracy.

## Drive choice and acceptance evidence

| Option | Mill assessment | Hobber assessment |
| --- | --- | --- |
| Basic V/f inverter, rpm command only | Possible for restricted cuts, but not the preferred low-speed/ATC platform | Does not meet the measured-angle synchronization requirement by itself |
| Sensorless vector drive | Candidate for ordinary milling; verify load response and low-speed cooling | May drive B in velocity mode if independent spindle feedback and C synchronization meet all bounds |
| Encoder-controlled vector spindle drive | Preferred sourcing architecture with the required motor curve and CNC functions | Preferred B-drive candidate; EGB and load-side C feedback are still separate requirements |
| AC servo and matching drive | Viable if speed/continuous torque/packaging fit; modeled small candidates fail the chosen output point | Viable with the same EGB; existing ELM rows are speed-limited for direct-drive B |

Closed-loop vector control and position-related functions exist in industrial
drives; this does not imply every VFD or firmware option provides them.
[Yaskawa A1000 capabilities](https://www.yaskawa.com/products/drives/industrial-ac-drives/general-purpose-drives/a1000-drive).
Haas documents vector spindle control and synchronized tapping as an industrial
example, not proof that a generic inverter supports those functions.
[Haas spindle systems](https://www.haascnc.com/productivity/spindles.html).
Load-side rotary feedback addresses errors invisible to motor encoders.
[HEIDENHAIN rotary-axis positioning accuracy](https://www.heidenhain.com/fileadmin/pdf/en/01_Products/Technische_Dokumentation/TI_Positioning_accuracy_of_rotary_axes_en.pdf).

Before selecting hardware, collect a continuous torque-speed curve with cooling
conditions, permissible overspeed and shaft/pulley loads, supply/drive current
requirements, thermal duty and braking energy. For the mill, redo CAD and moving
payload checks against the chosen motor drawing and confirm spindle orientation
and release interlocks. For the hobber, obtain installed encoder accuracy and
timing specifications, verify encoder capture bandwidth at maximum rpm, and
demonstrate the 1 µm phase-error allocation using an independent measurement
reference or reconciled metrology uncertainty. Stop/restart, reversal, missed
samples, loss of synchronization and loaded acceleration require explicit tests.

The present outcome is **conditional architecture selection**, not a purchase
list. The mill's first sourcing target is a 3 kW class spindle-duty vector system;
the hobber's is an 8000 rpm, 1.5 kW useful-output spindle plus the specified
measured-angle EGB and C-axis servo. Actual cutting requirements, supply and
manufacturer evidence can change either target. Existing CAD and runtime bindings
remain as they were; these requirements do not add a VFD simulation or implement HB7.

## Reproduce

From the repository root:

```sh
python3 machinekit/examples/spindle-drive-study/analyze.py --check
python3 machinekit/examples/spindle-drive-study/analyze.py --plot
```

The first command checks the committed numeric results and reference identities;
the second regenerates results and the standalone figure (matplotlib required).
No external Python packages are required for numerical calculation.
