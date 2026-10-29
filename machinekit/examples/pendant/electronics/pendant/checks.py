"""Pendant board checks: the shared read-only checks plus the pendant's own (its pin table, its 5 V rail).

`GATES` grades the status phases this board can check today; placement and later phases wait for the enclosure envelope
(EN3) and stay UNEXAMINED. `check_board` is what export runs.
"""

import functools
from pathlib import Path

from pcbkit import defaults, parts, status
from pcbkit.project_checks import Context

HERE = Path(__file__).resolve().parent


def context(**kwargs):
    return Context(HERE, "pendant", libraries=defaults.libraries(HERE), **kwargs)


def _value(code, *path):
    node = parts.load(code)["data"]
    for key in path:
        node = node[key]
    return node["value"]


def five_volt_rail(ctx):
    """The buck's divider holds 5 V inside what its loads accept, over the reference and resistor tolerances.

    The loads are the isolated CAN transceiver's VDDP (the tightest) and the LDO; the wheel takes whatever 5 V the board
    has. Values come from the part records, so a part swap re-checks itself."""
    by_tag = ctx.nl.by_tag()
    top, bottom = by_tag.get("power/r_fb_top"), by_tag.get("power/r_fb_bot")
    if top is None or bottom is None or "LCSC" not in top.fields or "LCSC" not in bottom.fields:
        return ["5 V rail: the feedback divider power/r_fb_top, power/r_fb_bot is missing"]
    buck = by_tag["power/u_buck"].fields["LCSC"]
    vref = (_value(buck, "behavior", "vref_min_v"), _value(buck, "behavior", "vref_max_v"))
    rt, rb = (_value(p.fields["LCSC"], "resistance_ohm") for p in (top, bottom))
    tt, tb = (_value(p.fields["LCSC"], "tolerance_pct") / 100 for p in (top, bottom))
    low = vref[0] * (1 + rt * (1 - tt) / (rb * (1 + tb)))
    high = vref[1] * (1 + rt * (1 + tt) / (rb * (1 - tb)))
    can = by_tag["can/u_can"].fields["LCSC"]
    accept = (_value(can, "recommended", "vddp_min_v"), _value(can, "recommended", "vddp_max_v"))
    if low < accept[0] or high > accept[1]:
        return [
            f"5 V rail: {low:.3f} to {high:.3f} V leaves the transceiver's VDDP range {accept[0]} to {accept[1]} V"
        ]
    return []


def circuit(ctx):
    return five_volt_rail(ctx) + ctx.pin_assignment().errors + ctx.pin_maps().errors


def electrical(ctx):
    blocks, gates = ctx.blocks(), ctx.gates()
    return status.Observation(
        errors=circuit(ctx) + blocks.errors + gates.errors,
        unknowns=blocks.unknowns + gates.unknowns,
    )


@functools.cache
def board():
    return context()


GATES = {
    "parts/locked_parts": lambda: board().parts(),
    "circuit/lint": lambda: board().lint(),
    "circuit/erc": lambda: board().erc(),
    "circuit/electrical": lambda: electrical(board()),
    "power/power": lambda: board().power(),
}


def check_board(nl, doc, pcb):
    ctx = context(nl=nl, doc=doc, pcb=pcb)
    errors = circuit(ctx)
    for name in ("parts", "lint", "erc", "blocks", "gates", "power"):
        errors += getattr(ctx, name)().errors
    return errors


if __name__ == "__main__":
    ctx = context()
    failures = circuit(ctx)
    for name in ("parts", "lint", "erc", "blocks", "gates", "power"):
        observation = getattr(ctx, name)()
        print(f"{name}: {len(observation.errors)} failure(s), {len(observation.unknowns)} unexamined")
        failures += observation.errors
    for error in failures:
        print("FAIL " + error)
    raise SystemExit(bool(failures))
