package machinekit.assembly;

import materia.assembly.AssemblyDefinition;
import machinekit.component.MachineComponent;

typedef AssemblySet = {
  final id:String;
  final label:String;
  final members:Array<String>;
}

/** Cross-cutting selections. IDs and order are deterministic; no hand-maintained member lists. */
class AssemblySets {
  public static function derive(definition:AssemblyDefinition):Array<AssemblySet> {
    var flat = materia.assembly.AssemblyDefinitionFlattener.flattenView(definition);
    var result:Array<AssemblySet> = [];
    var categories:Map<String, Array<String>> = [];
    for (occurrence in flat.occurrences) if (occurrence.selectionKinds != null)
      for (kind in occurrence.selectionKinds) {
        if (!categories.exists(kind)) categories.set(kind, []);
        categories.get(kind).push(occurrence.id);
      }
    var names = [for (kind in categories.keys()) kind];
    names.sort(Reflect.compare);
    for (kind in names) result.push({id: "kind:" + kind, label: kind, members: categories.get(kind)});
    var parents:Map<String, String> = [];
    for (joint in flat.joints) if (joint.role == "tree") parents.set(joint.child, joint.parent);
    for (joint in flat.joints) if (joint.role == "tree" && joint.type != "fixed") {
      var members:Array<String> = [];
      for (occurrence in flat.occurrences) {
        var cursor = occurrence.id, visited:Map<String, Bool> = [];
        while (!visited.exists(cursor)) {
          if (cursor == joint.child) { members.push(occurrence.id); break; }
          visited.set(cursor, true);
          if (!parents.exists(cursor)) break;
          cursor = parents.get(cursor);
        }
      }
      result.push({id: "moves:" + joint.id, label: "Moves with " + joint.id, members: members});
    }
    for (selection in result) selection.members.sort(Reflect.compare);
    result.sort((a, b) -> Reflect.compare(a.id, b.id));
    return result;
  }

  /** Category rules use BOM recipe identity, procurement and typed electrical ports, never instance names. */
  public static function kinds(component:MachineComponent):Array<String> {
    var bom = component.bom, result:Array<String> = [];
    var identity = ((bom.typeId == null ? "" : bom.typeId) + "." + (bom.familyId == null ? "" : bom.familyId)).toLowerCase();
    function has(tokens:Array<String>):Bool {
      for (token in tokens) if (identity.indexOf(token) >= 0) return true;
      return false;
    }
    var standardFastener = StringTools.startsWith(identity, "machinekit.standard.") && has(["screw", "nut", "washer", "bolt"]);
    if (standardFastener || Std.isOfType(component, machinekit.standard.ScrewFastener)
      || Std.isOfType(component, machinekit.standard.HexNut) || Std.isOfType(component, machinekit.standard.FinePitchNut)
      || Std.isOfType(component, machinekit.standard.FlatWasher) || has(["slide-in-slot-nut"])) result.push("Fasteners");
    if (has(["dowel"])) result.push("Dowels");
    if (has(["cover", "wiper", "bellows", "guard"])) result.push("Covers and wipers");
    if (has(["casting", "cast-", "mineral", "insert"])) result.push("Castings and inserts");
    switch bom.procurement {
      case Buy(_, _, _): result.push("Purchased");
      case Make(_): result.push("Made");
      default:
        if (machinekit.component.IncludedInPurchase.of(component) != null) result.push("Purchased");
    }
    var electrical = component.axisMotor() != null || component.motorDrive() != null || component.electricalSource() != null;
    for (port in component.ports()) switch port.kind {
      case ElectricalPower, Signal: electrical = true;
      default:
    }
    if (electrical) result.push("Electrical");
    return result;
  }
}
