"""Register live phase checks here; absent gates remain UNEXAMINED."""

from pathlib import Path
from pcbkit import status


def project():
    return status.Project(Path(__file__).resolve().parent, "pendant")


if __name__ == "__main__":
    raise SystemExit(status.main(project()))
