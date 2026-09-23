# Editor responsiveness benchmark (2026-09-23)

## Method

An isolated worktree at `47833352` produced two HashLink bytecode builds. The
baseline restored `BuildContext`, `KeyScope`, `StateStore`, `ComputedStyle`, and
`StyleResolver` from `47833352^`; the optimized build used those five files from
`47833352`. Capture tooling and native libraries were identical. Both builds
used `haxeon build --compiler-only`; the worktree used the same native libraries
for both variants.

Each normal capture lasted four seconds and replayed Hierarchy, Sensors, and an
inspector rename. All six runs of each variant verified the committed name in
the saved scene. Runs were interleaved to reduce time-of-run bias. Values below
are medians of per-run medians. The captures are in the ignored local directory
`app/build/profiles/ab-ui-cache-20260923/`.

| Normal event-driven capture | Baseline | Optimized |
| --- | ---: | ---: |
| Frame time | 35.9 ms | 30.1 ms |
| Tree and style time | 32.4 ms | 26.1 ms |
| Process CPU over four seconds | 1.70 s | 1.44 s |
| Sensors action frame | 47.2 ms | 43.2 ms |
| Inspector focus frame | 39.9 ms | 47.2 ms |

These interaction samples are small and individual frame times overlap. The
optimized build used less CPU in all six paired runs, while the inspector focus
frame did not improve consistently. Most measured frame time is in tree and
style work.

## Continuous redraw stress capture

This separate 180-frame capture deliberately requests every next frame. It
measures sustained allocation pressure, not normal idle behavior. The
unprofiled frame median was 26.1 ms before and 18.7 ms after the cache change.
The profiler adds substantial overhead, so its numbers below compare only
profiled runs:

| Profiled 180-frame capture | Baseline | Optimized |
| --- | ---: | ---: |
| Estimated HashLink allocations | 1,901 MiB | 1,151 MiB |
| GC collections | 175 | 116 |
| GC mark time | 7,649 ms | 4,382 ms |

The existing cache change materially reduces allocation and GC pressure under
continuous redraw, but it does not eliminate it. HashLink's allocation samples
still identify string concatenation as the largest allocation source; they do
not attribute those samples to a specific Haxe caller. A local `StringBuf`
change to style cache-key construction did not improve the unprofiled stress
capture and increased profiled estimated allocations to 1,250 MiB, so it was
discarded.

## Remaining cost

The new per-frame counters found slow frames even when all 205 style
resolutions were cache hits. One such frame took 57 ms, with no cache misses or
changed style nodes. This points to the cost of constructing and traversing the
UI tree and building cache keys on every update. The next optimization should
measure which subtrees account for that work, then reduce rebuild scope with
explicit invalidation boundaries. Another broad cache would add complexity
without evidence that style cascade misses cause the remaining lag.
