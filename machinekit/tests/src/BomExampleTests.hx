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
    var pressBlock = new PressBrake.PressBlock("test bed", 100, 80, 60).bom;
    var curtain = new PressBrake.PressLightCurtain().bom;
    if (pressBlock.procurement == null || pressBlock.maturity != "Assumed" || pressBlock.requirements == null
      || pressBlock.requirements.length != 3 || curtain.procurement == null || curtain.maturity != "Assumed")
      throw "Press-brake reference parts must declare their selection inputs";
    check("bench-mill", new BenchMill());
    check("cnc-router", new CncRouter());
    check("robot-welder", new WeldingCell());
    check("cobot-arm", new machinekit.robotics.CobotArm(machinekit.robotics.CobotClass.Reach850));
    check("press-brake", new PressBrake());
    check("robot-screwdriver", new ScrewdrivingCell());
    check("flexspline-hobber", new FlexsplineHobber());
  }

  static function check(name:String, machine:MachineAssembly):Void {
    var document = machine.bomDocument(name, "test-commit", "2026-01-01T00:00:00Z");
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
    if (name == "robot-screwdriver" && countIncludedPrefix(document.assembly, "CUSTOM-SCREWDRIVER-MOTOR-GEARBOX-") != 2)
      throw "The screwdriver slide and carrier must be included in its purchase";
    if (name == "cnc-router") {
      var yBracket = [for (line in document.procurement)
        if (line.item.partNumber == "BRACKET-HOME-SWITCH-BRACKET-Y") line];
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
