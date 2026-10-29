"""Decisions source; replace the bootstrap record with the product's facts."""

from pathlib import Path
from pcbkit.scaffold import record_main


def build():
    return {
        "schema": 1,
        "board": "pendant",
        "decisions": [
            {
                "id": "DEC-SPEC",
                "question": "What are this board's sourced functional and mechanical requirements?",
                "options": ["Define and review the product requirements"],
                "choice": None,
                "rationale": "Bootstrap only; no circuit, process or mechanical proposal "
                "is approved.",
                "decided_by": None,
                "date": None,
                "affects": ["REQ-SPEC-1"],
                "status": "open",
                "blocks": ["spec", "export", "order"],
                "source": {
                    "placeholder": "New-board bootstrap: product requirements "
                    "have not been defined or reviewed"
                },
            }
        ],
        "reviews": {},
    }


if __name__ == "__main__":
    raise SystemExit(record_main(Path(__file__).with_suffix(".json"), build, "decisions"))
