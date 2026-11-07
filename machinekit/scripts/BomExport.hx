import machinekit.assembly.MachineAssembly;

private class BomExample {
  public final name:String;
  public final create:Void -> MachineAssembly;
  public function new(name:String, create:Void -> MachineAssembly) {
    this.name = name;
    this.create = create;
  }
}

/** Builds a named example without running its preview or interference checks. */
class BomExport {
  public static function main():Void {
    var args = Sys.args();
    if (args.length == 1 && args[0] == "--occurrence-names") {
      OccurrenceNamesAudit.main();
      return;
    }
    if (args.length != 4) throw "Expected machine, source commit, generation time and output path";
    var machine = build(args[0]);
    var releasePath = "machinekit/examples/" + args[0] + "/releases.json";
    var releases = sys.FileSystem.exists(releasePath)
      ? machinekit.component.BomReleases.parse(sys.io.File.getContent(releasePath)) : null;
    sys.io.File.saveContent(args[3], haxe.Json.stringify(machine.bomDocument(args[0], args[1], args[2], releases), null, "  ") + "\n");
  }

  /** The names supported by the command and by example-BOM validation. */
  public static function names():Array<String> return [for (example in examples()) example.name];

  public static function build(name:String):MachineAssembly {
    for (example in examples()) if (example.name == name) return example.create();
    throw 'Unknown example machine "$name"';
  }

  // Created on demand, so factories never initialise another class's statics at load time.
  static function examples():Array<BomExample> return [
    new BomExample("bench-mill", () -> new BenchMill()),
    new BomExample("enclosed-bench-mill", () -> new EnclosedBenchMill()),
    new BomExample("cnc-router", () -> new CncRouter()),
    new BomExample("sheet-router", () -> new SheetRouter()),
    new BomExample("robot-welder", () -> new WeldingCell()),
    new BomExample("mobile-welder", () -> new MobileWelderCell()),
    new BomExample("robot-arm", () -> new RobotArm()),
    new BomExample("mobile-base", () -> new MobileBase()),
    new BomExample("mobile-base-cell", () -> new MobileBaseCell()),
    new BomExample("gantry-picker", () -> new GantryPicker()),
    new BomExample("gantry-welder", () -> new GantryWelder()),
    new BomExample("track-arm", () -> new TrackWelder()),
    new BomExample("overhead-arm", () -> new OverheadArm()),
    new BomExample("machine-tending", () -> new TendingControllers()),
    new BomExample("xyz-stage", () -> new XyzStage()),
    new BomExample("linear-stage", () -> LinearStage.axis()),
    new BomExample("cobot-arm", () -> new machinekit.robotics.CobotArm(machinekit.robotics.CobotClass.Reach850)),
    new BomExample("picking-station", pickingStation),
    new BomExample("corexy-plotter", () -> new CoreXyPlotter()),
    new BomExample("corexy-printer", () -> new CoreXyPrinter()),
    new BomExample("press-brake", () -> new PressBrake()),
    new BomExample("flexspline-hobber", () -> new FlexsplineHobber()),
    new BomExample("robot-screwdriver", () -> new ScrewdrivingCell())
  ];

  static function pickingStation():MachineAssembly {
    return new pickingstation.PickingStation().assembly();
  }
}
