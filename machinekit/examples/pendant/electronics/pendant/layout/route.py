"""Route through the shared pcbkit command."""

from pathlib import Path
from pcbkit.routing_run import main

if __name__ == "__main__":
    raise SystemExit(main(Path(__file__).resolve().parents[1], "pendant"))
