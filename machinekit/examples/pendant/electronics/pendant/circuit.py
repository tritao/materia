"""Circuit and intent skeleton; select a reviewed process before adding parts."""

from pathlib import Path
from pcbkit.board import Board
from pcbkit.intent import Intent

HERE = Path(__file__).resolve().parent


def build(out_dir=HERE, refs_path=HERE / "refs.lock"):
    board = Board("pendant", refs_path)
    intent = Intent(board.id)
    intent.workflow(
        requirements="machinekit/examples/pendant/electronics/pendant/requirements.json",
        decisions="machinekit/examples/pendant/electronics/pendant/decisions.json",
        netlist="machinekit/examples/pendant/electronics/pendant/pendant.net",
        parts_lock="machinekit/examples/pendant/electronics/pendant/parts.lock.json",
    )
    # No process, outline or page is invented. Fill these from the runbook.
    net_path = board.write(out_dir)
    intent.write(Path(out_dir) / (board.id + ".intent.json"), net_path)
    return net_path


if __name__ == "__main__":
    build()
