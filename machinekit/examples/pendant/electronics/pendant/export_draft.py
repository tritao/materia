"""Checked local export; a skeleton has no manufacturing/order configuration."""

from pathlib import Path
from checks import check_board
from pcbkit.export_run import main

if __name__ == "__main__":
    raise SystemExit(main(Path(__file__).resolve().parent, "pendant", check_board=check_board))
