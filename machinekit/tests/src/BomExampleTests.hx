import machinekit.assembly.MachineAssembly;

/** Exercise real machine exports without the unrelated preview/interference checks. */
class BomExampleTests {
  public static function main():Void run();
  public static function run():Void {
    check("bench-mill", new BenchMill());
    check("cnc-router", new CncRouter());
    check("robot-welder", new WeldingCell());
    check("flexspline-hobber", new FlexsplineHobber());
  }

  static function check(name:String, machine:MachineAssembly):Void {
    var document = machine.bomDocument(name, "test-commit", "2026-01-01T00:00:00Z");
    var json:Dynamic = haxe.Json.parse(haxe.Json.stringify(document));
    if (json.schema != "machinekit.bom" || json.version != 1 || json.machineId != name
      || json.sourceCommit != "test-commit" || json.assembly.path != "") throw 'Invalid $name BOM document';
    if (document.procurement.length == 0) throw 'Empty $name BOM';
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
}
