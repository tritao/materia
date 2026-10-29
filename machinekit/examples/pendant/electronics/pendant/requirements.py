"""Requirements source; replace the bootstrap record with the product's facts."""

from pathlib import Path
from pcbkit.scaffold import record_main


def build():
    return {
        "schema": 1,
        "board": "pendant",
        "requirements": [
            {
                "id": "REQ-SPEC-1",
                "text": "Replace the bootstrap with sourced product requirements "
                "before designing the board.",
                "source": {
                    "placeholder": "New-board bootstrap: product requirements "
                    "have not been defined or reviewed"
                },
                "verified_by": ["check:skeleton"],
            }
        ],
        "verifications": [
            {
                "id": "check:skeleton",
                "kind": "check",
                "text": "Refuse an unfinished new-board skeleton",
                "source": {
                    "file": "machinekit/examples/pendant/electronics/pendant/checks.py",
                    "where": "unfinished",
                },
            }
        ],
    }


if __name__ == "__main__":
    raise SystemExit(record_main(Path(__file__).with_suffix(".json"), build, "requirements"))
