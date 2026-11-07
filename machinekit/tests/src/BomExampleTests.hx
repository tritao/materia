import machinekit.assembly.MachineAssembly;

/** Exercise real machine exports without the unrelated preview/interference checks. */
class BomExampleTests {
  public static function main():Void run();
  public static function run():Void {
    var bracket = new BenchMill.MillDriveBox("MILL-TEST-BRACKET", 20, 30, 10);
    var revised = new BenchMill.MillDriveBox("MILL-TEST-BRACKET", 20, 30, 10.001);
    if (bracket.bom.partNumber != revised.bom.partNumber || bracket.bom.fingerprint == null
      || bracket.bom.fingerprint == revised.bom.fingerprint)
      throw "A bracket geometry change must preserve its drawing number and change its fingerprint";
    var lidar = new MobileBase.LidarPuck().bom;
    if (lidar.maturity != "Assumed" || lidar.requirements == null
      || [for (term in lidar.requirements) if (term.name == "range" && term.min == "6") term].length != 1)
      throw "The mobile lidar must carry its assumed range";
    for (recipe in pickingstation.PickingStationRecipes.all()) {
      var item = recipe.create().bom;
      if (item.procurement == null || item.maturity == null)
        throw 'Picking station recipe is missing procurement intent: ${recipe.id}';
    }
    var weldingLead = machinekit.welding.WeldingServiceStock.powerLead(false, 3, 35);
    var revisedLead = machinekit.welding.WeldingServiceStock.powerLead(false, 4, 35);
    var weldingTerms = weldingLead.requirements;
    if (weldingLead.maturity != "Assumed" || weldingTerms == null || weldingTerms.length != 2
      || weldingLead.fingerprint == null || weldingLead.fingerprint == revisedLead.fingerprint)
      throw "Welding leads need structured selection dimensions and a fingerprint";
    var pack = machinekit.welding.WeldingServiceStock.torchPack("ROBOT", 1.5);
    var packTerms = pack.requirements;
    if (packTerms == null || packTerms.length != 5)
      throw "A torch pack must declare each modeled service separately";
    var pressBlock = new PressBrake.PressBlock("TEST-PRESS-BED", "test bed", 100, 80, 60).bom;
    var curtain = new PressBrake.PressLightCurtain().bom;
    if (pressBlock.procurement == null || pressBlock.maturity != "Assumed" || pressBlock.requirements == null
      || pressBlock.requirements.length != 3 || curtain.procurement == null || curtain.maturity != "Assumed")
      throw "Press-brake reference parts must declare their selection inputs";
    check("bench-mill", new BenchMill());
    check("cnc-router", new CncRouter());
    check("cnc-router-belt", new CncRouter(true));
    check("cnc-router-folded-z", new CncRouter(false, true));
    check("robot-welder", new WeldingCell());
    check("cobot-arm", new machinekit.robotics.CobotArm(machinekit.robotics.CobotClass.Reach850));
    check("press-brake", new PressBrake());
    check("robot-screwdriver", new ScrewdrivingCell());
    check("enclosed-bench-mill", new EnclosedBenchMill());
    check("mobile-welder", new MobileWelderCell());
    check("robot-arm", new RobotArm());
    check("mobile-base", new MobileBase());
    check("mobile-base-cell", new MobileBaseCell());
    check("gantry-picker", new GantryPicker());
    check("gantry-welder", new GantryWelder());
    check("track-arm", new TrackWelder());
    check("overhead-arm", new OverheadArm());
    check("xyz-stage", new XyzStage());
    check("linear-stage", LinearStage.axis());
    check("picking-station", new pickingstation.PickingStation().assembly());
    check("corexy-plotter", new CoreXyPlotter());
    check("corexy-printer", new CoreXyPrinter());
    check("sheet-router", new SheetRouter());
    check("flexspline-hobber", new FlexsplineHobber());
  }

  static function checkNumbers(machine:String, node:machinekit.assembly.BomDocument.BomAssemblyNode):Void {
    for (line in node.lines) {
      if (line.item.fingerprint == null)
        throw '$machine BOM at ${line.uses[0].path}: missing fingerprint for ${line.item.partNumber}';
      var problems = machinekit.component.PartNumberLint.problems(line.item.partNumber);
      if (problems.length != 0)
        throw '$machine BOM at ${line.uses[0].path}: ${line.item.partNumber}: ${problems.join(", ")}';
    }
    for (child in node.children) checkNumbers(machine, child);
  }

  static function check(name:String, machine:MachineAssembly):Void {
    machinekit.assembly.OccurrenceNames.requireReadable([for (occurrence in machine.definition().occurrences) occurrence.id]);
    var document = machine.bomDocument(name, "test-commit", "2026-01-01T00:00:00Z");
    checkNumbers(name, document.assembly);
    var json:Dynamic = haxe.Json.parse(haxe.Json.stringify(document));
    if (json.schema != "machinekit.bom" || json.version != 1 || json.machineId != name
      || json.sourceCommit != "test-commit" || json.assembly.path != "") throw 'Invalid $name BOM document';
    if (document.procurement.length == 0) throw 'Empty $name BOM';
    if (name == "bench-mill") {
      var brakes = [for (line in document.procurement) if (StringTools.startsWith(line.item.partNumber, "BRAKE-ASSUMED-4NM-")) line];
      if (brakes.length != 1 || brakes[0].item.quantity.value != 1)
        throw "The bench mill shaft brake must be one purchased unit";
      var owner = brakes[0].item.partNumber;
      var included = countIncluded(document.assembly, owner);
      if (included != 27) throw 'The brake engineering tree must retain its disc and sectors, got $included';
      var railHoles = 0;
      for (entry in machine.components()) if (Std.isOfType(entry.component, machinekit.motion.LinearRail)) {
        var rail:machinekit.motion.LinearRail = cast entry.component;
        railHoles += rail.holePositions.length;
      }
      var blocks = 0;
      for (entry in machine.components()) if (Std.isOfType(entry.component, machinekit.motion.LinearRailBlock)) blocks++;
      // Four screws per carriage block join the rail screws in the same M4 x 16 line.
      // The six covers add 16 M5x12 and 56 M5x10 flange screws, each with a washer.
      if (quantity(document, "ISO4762-M4x16") != railHoles + 4 * blocks || railHoles == 0
        || quantity(document, "ISO4762-M5x12") != 8 + 16 || quantity(document, "ISO4762-M6x12") != 4
        || quantity(document, "ISO4762-M5x10") != 56 || quantity(document, "ISO7089-M5") != 18 + 72
        || quantity(document, "ISO4762-M5x25") != 12 || quantity(document, "ISO4762-M5x20") != 4
        || quantity(document, "ISO8734-4x8") != 16)
        throw "Bench mill fasteners must match the rail, motor and cover flange patterns";
      var mill:BenchMill = cast machine;
      for (drive in mill.axisDrives) {
        var id = 'motor${drive.spec.id.toUpperCase()}Plate';
        var matches = [for (entry in machine.components()) if (entry.id == id) entry.component];
        if (matches.length != 1 || !Std.isOfType(matches[0], BenchMill.MillMotorPlate)) throw 'Missing $id';
        var plate:BenchMill.MillMotorPlate = cast matches[0];
        var motorFlange = drive.motor.mounting();
        if (plate.flange == null || motorFlange == null || plate.flange.holeDiameter != motorFlange.holeDiameter
          || haxe.Json.stringify(plate.flange.holes) != haxe.Json.stringify(motorFlange.holes))
          throw 'Motor plate $id must use its motor flange pattern';
      }
      var power = [for (line in document.procurement) if (line.item.partNumber == "MILL-POWER-CABLE-4C-400V-ASSUMED") line];
      var encoder = [for (line in document.procurement) if (line.item.partNumber == "MILL-ENCODER-CABLE-ASSUMED") line];
      var chains = [for (line in document.procurement) if (StringTools.startsWith(line.item.partNumber, "CHAIN-ASSUMED-")) line];
      var chainCuts = 0;
      for (line in chains) for (cut in line.item.cuts) chainCuts += cut.count;
      if (power.length != 1 || power[0].item.quantity.unit != "m" || power[0].item.cuts.length != 4
        || encoder.length != 1 || encoder[0].item.quantity.unit != "m" || encoder[0].item.cuts.length != 3
        || chainCuts != 3 || quantity(document, "MILL-LUBE-15-OUT-ASSUMED") != 1)
        throw "The mill must declare its cable cuts, three chains and lubrication set";
    }
    if (name == "robot-welder") {
      var workpiece = findUse(document.assembly, "work/basePlate");
      if (workpiece == null || workpiece.procurement == null || workpiece.procurement.kind != "NotSupplied")
        throw "The weldment workpiece must remain in the engineering BOM as not supplied";
      for (line in document.procurement) if (line.item.partNumber == workpiece.partNumber)
        throw "The weldment workpiece must not appear in the procurement view";
      if (countIncludedPrefix(document.assembly, "WELD-TORCH-") != 2)
        throw "The torch neck and nozzle must be included in the torch purchase";
    }
    if (name == "cobot-arm" && countIncludedPrefix(document.assembly, "COBOT-ASSUMED-") != 6)
      throw "Each cobot gearhead must be included in its joint purchase";
    if (name == "press-brake" && countIncludedPrefix(document.assembly, "HYD-CYL-") != 2)
      throw "The hydraulic cylinder rods must be included in their cylinder purchases";
    if (name == "robot-screwdriver") {
      var body = findUse(document.assembly, "arm/tool/driver/body");
      var slide = findUse(document.assembly, "arm/tool/driver/slide");
      var carrier = findUse(document.assembly, "arm/tool/driver/carrier");
      if (body == null || slide == null || carrier == null) throw "Missing screwdriver body, slide or carrier";
      if (slide.includedIn != body.partNumber || carrier.includedIn != body.partNumber
        || countIncludedPrefix(document.assembly, body.partNumber) != 2)
        throw "The screwdriver slide and carrier must be included in its purchase";
    }
    if (name == "cnc-router") {
      var yBracket = [for (line in document.procurement)
        if (line.item.typeId == "machinekit.structural.block-bracket"
          && line.item.description == "Home switch bracket y") line];
      if (yBracket.length != 1 || yBracket[0].item.quantity.value != 2)
        throw "Identical Y home-switch brackets must roll up to two pieces";
    }
    for (line in document.procurement) {
      var total = 0.0;
      for (use in line.uses) {
        if (use.quantity.unit != line.item.quantity.unit) throw 'Mismatched BOM use unit in $name';
        total += use.quantity.value;
      }
      if (Math.abs(total - line.item.quantity.value) > 0.00000001) throw 'BOM use quantity mismatch in $name';
    }
  }

  static function findUse(node:machinekit.assembly.BomDocument.BomAssemblyNode,
    path:String):Null<machinekit.assembly.BomDocument.BomDocumentItem> {
    for (line in node.lines) for (use in line.uses) if (use.path == path) return line.item;
    for (child in node.children) {
      var found = findUse(child, path);
      if (found != null) return found;
    }
    return null;
  }

  static function quantity(document:machinekit.assembly.BomDocument, number:String):Float {
    for (line in document.procurement) if (line.item.partNumber == number) return line.item.quantity.value;
    return 0;
  }

  static function countIncluded(node:machinekit.assembly.BomDocument.BomAssemblyNode, owner:String):Int {
    var count = 0;
    for (line in node.lines) if (line.item.includedIn == owner) count++;
    for (child in node.children) count += countIncluded(child, owner);
    return count;
  }

  static function countIncludedPrefix(node:machinekit.assembly.BomDocument.BomAssemblyNode, prefix:String):Int {
    var count = 0;
    for (line in node.lines) if (line.item.includedIn != null && StringTools.startsWith(line.item.includedIn, prefix)) count++;
    for (child in node.children) count += countIncludedPrefix(child, prefix);
    return count;
  }
}
