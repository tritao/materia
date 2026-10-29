"""Live phase checks for the pendant board; phases without a check stay UNEXAMINED."""

import json
from pathlib import Path

import decisions
import requirements
from pcbkit import status, workflow
from pcbkit.project_checks import Context

HERE = Path(__file__).resolve().parent


def records():
    """The WF1/WF2 records are valid against the intent and match their Python sources."""
    ctx = Context(HERE, "pendant")
    errors = workflow.problems(ctx.doc, {"reviews": {}}, ctx.nl)
    for kind, source in (("requirements", requirements), ("decisions", decisions)):
        if json.loads((HERE / (kind + ".json")).read_text()) != source.build():
            errors.append(kind + " record differs from its Python source")
    return status.Observation(errors=errors)


def project():
    return status.Project(HERE, "pendant", {"spec/requirements": records, "spec/decisions": records})


if __name__ == "__main__":
    raise SystemExit(status.main(project()))
