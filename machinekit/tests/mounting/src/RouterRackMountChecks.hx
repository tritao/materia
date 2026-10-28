/** Separate process per bed/check keeps native CAD within the router case cap. */
class RouterRackMountChecks {
  public static function main():Void {
    var args = Sys.args(), mode = args.length == 0 ? "contracts" : args[0];
    switch mode {
      case "contracts":
        AssemblyLargePartsTests.main();
        AxisBuilderChecks.run();
        AssemblyIntegrityTests.run();
        RealRackMountTests.main();
      case "pivot":
        RadialPinionTests.main();
      case "zero", "travel", "service", "model":
        var bed = args.length > 1 ? Std.parseFloat(args[1]) : 2600.0;
        if (bed != 2600 && bed != 3150) throw "Choose bed 2600 or 3150";
        var machine = new SheetRouter(new SheetRouterSpec(bed));
        machine.check().throwIfErrors();
        if (mode == "model") SheetRouterRackMountExport.write(
          machine,
          'build/router-mount-implementation/model-' + Std.int(bed)
        );
        else if (mode == "zero") RackMountClearanceChecks.atZero(machine, bed);
        else if (mode == "service") RackMountServiceChecks.run(machine, bed);
        else RackMountTravelChecks.run(machine, bed);
      case _:
        throw "Unknown rack mount check: " + mode;
    }
  }
}
