"""Replace this failing bootstrap with the product's check inventory and fixtures."""


def unfinished():
    return [
        "New-board skeleton: define sourced requirements, select a process and implement circuit checks"
    ]


def check_board(nl, doc, pcb):
    return unfinished()


CHECKS = {"skeleton": unfinished}


if __name__ == "__main__":
    for name, fn in CHECKS.items():
        for error in fn():
            print("FAIL " + name + ": " + error)
    raise SystemExit(1)
