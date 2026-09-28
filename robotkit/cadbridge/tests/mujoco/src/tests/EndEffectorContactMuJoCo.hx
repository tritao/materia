package tests;

import cadbridge.EndEffectorBridge;
import cadbridge.EndEffectorCollision;
import cadbridge.EndEffectorCollision.EndEffectorCollisionOptions;
import eoat.EndEffectorExample;
import haxe.Int64;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.Simulation;
import robotkit.tool.ToolCollisionShape;
import robotkit.tool.ToolCollisionShapes;

/** The example cup reports a proximity contact without touching other pieces. */
class EndEffectorContactMuJoCo {
  public static function main():Void {
    var effector = EndEffectorExample.build().configuration("short");
    var options = new EndEffectorCollisionOptions(10, 16, 10);
    var collision = EndEffectorCollision.pieces(effector, null, options);
    var cupIndex = -1;
    for (index in 0...collision.pieces.length)
      if (collision.pieces[index].memberIds.indexOf("tool/cup") >= 0) cupIndex = index;
    if (cupIndex < 0) throw "Example cup has no collision piece";
    var cup = ToolCollisionShapes.bounds(ToolCollisionShape.Hulls(
      [collision.pieces[cupIndex].vertices], 0));
    var tool = EndEffectorBridge.toTool(effector, "contact", null, "short/contact", options);
    var simulation = new Simulation(0.01, 2, 1);
    var runtime = simulation.addRobotAtPose(new RobotRuntimeBlueprint(1, 0, 1),
      [0, 0, 0], [0, 0, 0, 1], null, null, null, null, tool.collision, 0);
    var obstacle = simulation.spawnBox([
      cup.centre.x, cup.centre.y, cup.centre.z + cup.halfExtents.z + 0.007],
      [0.005, 0.005, 0.005]);
    simulation.step(Int64.ofInt(0));
    var contacts = runtime.toolProximity();
    if (contacts.length == 0) throw "Example cup proximity was not reported";
    for (contact in contacts)
      if (contact.toolPieceIndex != cupIndex || contact.otherObject != obstacle)
        throw "Proximity came from a piece other than the example cup";
    simulation.dispose();
    Sys.println("CadBridge MuJoCo cup proximity passed");
  }
}
