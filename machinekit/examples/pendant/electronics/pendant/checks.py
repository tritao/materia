"""Pendant board checks: the shared read-only checks plus the pendant's own pin table and supplier pin maps.

`GATES` grades the status phases this board can check today; placement and later phases wait for the enclosure envelope
(EN3) and stay UNEXAMINED. `check_board` is what export runs.
"""

import functools
from pathlib import Path

from pcbkit import defaults, status
from pcbkit.project_checks import Context

HERE = Path(__file__).resolve().parent


def context(**kwargs):
    return Context(HERE, "pendant", libraries=defaults.libraries(HERE), **kwargs)


def circuit(ctx):
    return ctx.pin_assignment().errors + ctx.pin_maps().errors


def electrical(ctx):
    blocks, gates, pinouts = ctx.blocks(), ctx.gates(), ctx.part_interfaces()
    return status.Observation(
        errors=circuit(ctx) + blocks.errors + gates.errors + pinouts.errors,
        unknowns=blocks.unknowns + gates.unknowns,
    )


def cable_exits(ctx):
    """Where the wires arrive (the envelope) and how each cable connector mates (its interface record's mechanics)."""
    envelope, pinouts = ctx.envelope(), ctx.part_interfaces()
    return status.Observation(errors=envelope.errors, unknowns=envelope.unknowns + pinouts.unknowns)


@functools.cache
def board():
    return context()


GATES = {
    "parts/locked_parts": lambda: board().parts(),
    "circuit/lint": lambda: board().lint(),
    "circuit/erc": lambda: board().erc(),
    "circuit/electrical": lambda: electrical(board()),
    "power/power": lambda: board().power(),
    # The case's envelope: stale when the case model changed since the intent was generated; its assumed parts are
    # unknowns, so these stay open until the parts are measured.
    "mechanical/mounting": lambda: board().envelope(),
    "mechanical/heights": lambda: board().envelope(),
    "mechanical/cable_exits": lambda: cable_exits(board()),
}


def check_board(nl, doc, pcb):
    ctx = context(nl=nl, doc=doc, pcb=pcb)
    errors = circuit(ctx)
    for name in ("parts", "lint", "erc", "blocks", "gates", "power", "part_interfaces"):
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
