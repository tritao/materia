import machinekit.assembly.MachineAssembly;
import machinekit.assembly.AssemblySets;
import machinekit.standard.HexNut;
import machinekit.standard.DowelPin;
import materia.assembly.AssemblyFrames as F;

class SceneGroupTests {
  static function expect(value:Bool, message:String):Void { if (!value) throw message; }
  public static function main():Void {
    var child = new MachineAssembly();
    child.group("Drive");
    child.addComponent("nut", HexNut.metric("M5"));
    child.group("Structure");
    child.addComponent("pin", DowelPin.metric("5", 16));
    var parent = new MachineAssembly();
    parent.group("X axis");
    parent.include("unit", child);
    parent.withGroup("Bed", () -> parent.addComponent("base", HexNut.metric("M5")));
    parent.addComponent("fixed", HexNut.metric("M5"));
    expect(parent.functionalGroups().get("fixed") == "X axis", "group scope restores previous group");
    expect(parent.functionalGroups().get("unit/nut") == "X axis/unit/Drive", "included path retains group and include identity");
    child.assignGroup("nut", "Changed");
    expect(parent.functionalGroups().get("unit/nut") == "X axis/unit/Drive", "include is an owned snapshot");
    var restored = MachineAssembly.decode(parent.encode());
    expect(restored.functionalGroups().get("unit/pin") == "X axis/unit/Structure", "wire preserves nested group ownership");
    expect(restored.checkGroups().items.length == 0, "all members grouped");
    expect(parent.sets().map(s -> s.id + ":" + s.members.join(",")).join("|") == restored.sets().map(s -> s.id + ":" + s.members.join(",")).join("|"), "derived set order survives description sorting");
    expect(haxeon.wire.JsonWire.encode(parent.describe()) == haxeon.wire.JsonWire.encode(restored.describe()), "description roundtrip is lossless");
    var expanded = materia.assembly.AssemblyDefinitionFlattener.flattenView(parent.describe().mechanical);
    expect(expanded.occurrences.filter(o -> o.id == "unit/nut")[0].groupPath == parent.functionalGroups().get("unit/nut"), "nested and flat exports agree on ownership");
    var fallback = new MachineAssembly(); fallback.addComponent("nut", HexNut.metric("M5"));
    parent.include("fallback", fallback);
    expect(parent.functionalGroups().get("fallback/nut") == "X axis/fallback" && parent.checkGroups().items.length == 0, "include supplies ownership to ungrouped descendants");
    // An include that is its group: no second folder named after the include, through encode/decode and the flat export.
    var unit = new MachineAssembly();
    unit.group("Drive");
    unit.addComponent("nut", HexNut.metric("M5"));
    var host = new MachineAssembly();
    host.includeAsGroup("Machine", "machine", unit);
    host.includeAsGroup("Shell", "shell", fallback);
    expect(host.functionalGroups().get("machine/nut") == "Machine/Drive", "include as group puts its groups directly under it");
    expect(host.functionalGroups().get("shell/nut") == "Shell", "include as group owns its ungrouped members");
    var hostRestored = MachineAssembly.decode(host.encode());
    expect(hostRestored.functionalGroups().get("machine/nut") == "Machine/Drive", "include as group survives the description");
    var hostFlat = materia.assembly.AssemblyDefinitionFlattener.flattenView(host.describe().mechanical);
    expect(hostFlat.occurrences.filter(o -> o.id == "machine/nut")[0].groupPath == "Machine/Drive", "flat export agrees with include as group");
    expect(hostFlat.occurrences.filter(o -> o.id == "shell/nut")[0].groupPath == "Shell", "flat export gives ungrouped members the include's group");
    var definition = restored.definition();
    definition.joints = [
      {id:"X", type:"prismatic", role:"tree", parent:"base", parentConnector:"front", child:"fixed", childConnector:"front",
        axis:{x:1.0,y:0.0,z:0.0}, limits:{lower:null,upper:null,velocity:null,effort:null}, defaultValue:0},
      {id:"mount", type:"fixed", role:"tree", parent:"fixed", parentConnector:"front", child:"unit/nut", childConnector:"front",
        axis:{x:1.0,y:0.0,z:0.0}, limits:{lower:null,upper:null,velocity:null,effort:null}, defaultValue:0}
    ];
    var sets = AssemblySets.derive(definition);
    var moving = sets.filter(s -> s.id == "moves:X")[0].members;
    expect(moving.length == 2 && moving.indexOf("fixed") >= 0 && moving.indexOf("unit/nut") >= 0, "moves-with follows fixed descendants only");
    expect(sets.filter(s -> s.id == "kind:Fasteners")[0].members.length == 3, "fasteners from BOM identity");
    expect(sets.filter(s -> s.id == "kind:Dowels")[0].members.join(",") == "unit/pin", "dowels from BOM identity");
    expect(sets.filter(s -> s.id == "kind:Purchased")[0].members.length == 4, "procurement category");
    var categories = new MachineAssembly(); categories.group("Fixtures");
    categories.addComponent("shield", new machinekit.hobbing.HobberChipCover());
    categories.addComponent("insert", new machinekit.structural.CastInsert(new machinekit.component.ComponentValues().setToken("role", materia.casting.InsertSpec.InsertRole.GenericMount)));
    categories.addComponent("motor", machinekit.motion.NemaStepper.frame(23));
    categories.addComponent("nutMount", new machinekit.milling.MillYNutMount());
    categories.addComponent("ballNut", new machinekit.motion.BallNut());
    categories.addComponent("rotor", new machinekit.hobbing.HobSpindlePart());
    var byKind = categories.sets();
    expect(byKind.filter(s -> s.id == "kind:Purchased")[0].members.indexOf("rotor") >= 0, "bundled physical subparts inherit their explicit purchase ownership");
    expect(byKind.filter(s -> s.id == "kind:Fasteners").length == 0, "ball nuts and nut brackets are motion parts, not fastening hardware");
    expect(byKind.filter(s -> s.id == "kind:Covers and wipers")[0].members.join(",") == "shield", "covers from recipe metadata");
    expect(byKind.filter(s -> s.id == "kind:Made")[0].members.indexOf("shield") >= 0, "made from procurement intent");
    expect(byKind.filter(s -> s.id == "kind:Castings and inserts")[0].members.join(",") == "insert", "castings and inserts from recipe metadata");
    expect(byKind.filter(s -> s.id == "kind:Electrical")[0].members.join(",") == "motor", "electrical from typed component interfaces");
    var plain = new MachineAssembly(); plain.addComponent("nut", HexNut.metric("M5"));
    expect(plain.checkGroups().items.length == 1 && plain.functionalGroups().get("nut") == "Ungrouped", "migration warning");
    var rejected = false;
    try plain.group("bad//path") catch (_:Dynamic) rejected = true;
    expect(rejected, "invalid paths rejected");
    rejected = false;
    var old = parent.describe(); old.schemaVersion--;
    try MachineAssembly.fromDescription(old) catch (_:Dynamic) rejected = true;
    expect(rejected, "old schema rejected");
    Sys.println("Scene group library tests passed (22 checks)");
  }
}
