@:access(WeldPlanningTests)
class WeldPlanningMain {
  public static function main():Void {
    WeldPlanningTests.run();
    WeldPassPathTests.run();
    var cell = WeldPlanningTests.cell(null);
    var planning = processkit.WeldingPlanRunner.planning(cast cell.clearance.arm, 2.0, cell.clearance);
    var planned = planning.planner.plan(WeldPlanningTests.seam(), cell.start);
    if (planned.checked == 0 || planned.plan.length() < 0.099) throw "Shared execution planner did not verify the full weld";
    var end = planning.compiler.solver.forward(planned.endJoints);
    if (Math.sqrt(Math.pow(end.x - planned.retreat.x, 2) + Math.pow(end.y - planned.retreat.y, 2) +
        Math.pow(end.z - planned.retreat.z, 2)) > 0.0005) throw "Verified final joints do not reach the retreat pose";
    Sys.println("Shared welding motion factory: complete compiled motion passes");
  }
}
