---
title: Conventions
description: Frame handedness, axes, units, quaternion order and transform naming, stated once for every kit.
status: stub
---

# Conventions

Frame handedness, axes, units, quaternion order and transform naming, stated once for every kit.

> **Stub.** The outline below lists what this page must cover. Replace each
> section with prose; keep the five headings.

## The problem

<!-- Why this concept exists: the question a reader has before reading. -->

## The model

<!-- One diagram and the core idea. No API detail. -->

To cover:

- Right-handed frames, +X forward, +Y left, +Z up.
- SI units: metres, radians, seconds, metres/second for feeds.
- Quaternions ordered `x,y,z,w`; transforms named `A_T_B` (maps coordinates in `B` into `A`).
- Known exceptions and why (e.g. `RingStock` and GearKit work in millimetres).
- No normative conventions document exists today; each kit restates these. This page becomes the one source.

## In Materia

<!-- Which kit owns it, the key types, links into the API reference. -->

## What it isn't

<!-- Boundaries and common confusions. -->

## See also

- [The life of a machine](../start/life-of-a-machine.md)

<!-- Sources to distil (remove this block when the page is written): -->
<!--
  visionkit/PLAN.md
  robotkit/CONSTRUCTION_ROADMAP.md
  motionkit/IMPLEMENTATION_PLAN.md
  animkit/README.md
  stockkit/README.md
-->
