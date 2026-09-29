package humanoid;

/** Humanoid simulation checks (robotkit/plans/HUMANOID.md): `drop`, `stand`, `walk`, `conformance` and `mixed`. */
class Main {
  public static function main():Void {
    var args = Sys.args();
    var command = args.length == 0 ? "" : args.shift();
    switch command {
      case "drop": DropCheck.run(args);
      case "conformance": Conformance.run(args);
      case "stand": StandCheck.run(args);
      case "mixed": MixedScene.run(args);
      case "walk": WalkCommand.run(args);
      case _:
        Sys.println("usage: drop <robot.json> [drop-height] | stand <robot.json> <poses.json> <pose> ... | conformance <robot.json> <reference.csv> ... | mixed <robot.json>");
        Sys.exit(2);
    }
  }
}
