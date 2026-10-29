"""Place through the shared pcbkit command."""

from pathlib import Path
from pcbkit.placement_run import main

if __name__ == "__main__":
    raise SystemExit(main(Path(__file__).resolve().parents[1], "pendant"))
