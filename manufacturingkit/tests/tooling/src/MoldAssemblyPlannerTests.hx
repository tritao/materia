import materia.tooling.casting.MoldAssemblyPlanner;
import materia.tooling.casting.MoldAssemblyPlanner.MoldAssemblyStep;
import materia.tooling.casting.MoldAssemblyPlanner.MoldAssemblyStepKind;

/** Pure graph regressions; no CAD or presentation runtime is needed. */
class MoldAssemblyPlannerTests {
  static var checks = 0;
  static function check(ok:Bool, label:String):Void {
    checks++;
    if (!ok) throw label;
  }
  static function step(id:String, kind:MoldAssemblyStepKind, dependencies:Array<String>):MoldAssemblyStep return {
    id: id,
    kind: kind,
    title: id,
    instruction: id,
    memberIds: [id],
    highlightIds: [id],
    dependsOn: dependencies,
    clearance: {x: 0, y: 0, z: 0},
    evidence: {kind: ManualOperation, checkedMember: null, blockers: [], travelMm: 0, toleranceMm3: 0}
  };
  static function rejects(steps:Array<MoldAssemblyStep>):Bool {
    try MoldAssemblyPlanner.order(steps) catch (_:Dynamic) return true;
    return false;
  }
  public static function main():Void {
    var source = [
      step("insert-b", Steel, ["support"]),
      step("closure", Panel, ["insert-b"]),
      step("insert-a", Steel, ["support"]),
      step("fix-a", Fastening, ["insert-a", "base"]),
      step("support", Tooling, ["wall"]),
      step("wall", Panel, ["base"]),
      step("base", Base, [])
    ];
    var original = [for (value in source) value.id].join(",");
    var ordered = MoldAssemblyPlanner.order(source), ids = [for (value in ordered) value.id];
    check(
      ids.join(",") == "base,wall,support,insert-a,fix-a,insert-b,closure",
      "Build supports first, promptly fasten their insert, and defer obstructing closure"
    );
    for (value in ordered) for (dependency in value.dependsOn) check(
      ids.indexOf(dependency) < ids.indexOf(value.id),
      "Every dependency precedes its operation"
    );
    check([for (value in source) value.id].join(",") == original, "Ordering leaves input records intact");
    source.reverse();
    check(
      [for (value in MoldAssemblyPlanner.order(source)) value.id].join(",") == ids.join(","),
      "Ordering is deterministic"
    );
    check(rejects([step("a", Panel, ["b"]), step("b", Steel, ["a"])]), "Reject cycles");
    check(rejects([step("a", Panel, ["missing"])]), "Reject unknown dependencies");
    check(rejects([step("a", Panel, []), step("a", Steel, [])]), "Reject duplicate steps");
    Sys.println('Mould assembly planner: $checks checks passed');
  }
}
