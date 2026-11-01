import machinekit.assembly.MachineAssembly;
import machinekit.bomschema.BomV1.Line;
import machinekit.component.BomJson;

/** Exercise the same JSON boundary used by sourcing consumers. */
class SourcingDocuments {
  public static function procurement(assembly:MachineAssembly):Array<Line> {
    var missing:Array<String> = [];
    for (entry in assembly.components()) {
      var item = entry.component.bom;
      if (item.procurement == null || item.maturity == null || item.fingerprint == null)
        missing.push(entry.id + ": " + item.partNumber);
    }
    if (missing.length > 0) throw "Incomplete supplier-example BOM: " + missing.join(", ");
    var document = assembly.bomDocument("supplier-check", "test", "2026-01-01T00:00:00Z");
    return BomJson.parse(haxe.Json.stringify(document)).procurement;
  }
}
