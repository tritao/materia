package processkit;

import collisionkit.CollisionPairRule;
import collisionkit.CollisionPairStatus;
import robotkit.collision.CollisionSceneBuilder;

/** Handling permissions name physical object sets, never assembly path conventions. */
class HandlingCollisionPolicy {
  public static function workInterfaces(scene:CollisionSceneBuilder, tools:Array<Int>, targets:Array<Int>,
      pads:Array<Int>, pressureTargets:Array<Int>, margin:Float):Void {
    for (tool in tools) for (target in targets) if (tool != target) {
      var pressure = pads.indexOf(tool) >= 0 && pressureTargets.indexOf(target) >= 0;
      scene.pair(scene.description.objects[tool].name, scene.description.objects[target].name,
        pressure ? CollisionPairRule.Allow : CollisionPairRule.Default,
        pressure ? CollisionPairStatus.ProcessContact : CollisionPairStatus.Checked, margin);
    }
  }

  /** A seated blank may touch its named seats and jaws; penetration remains a collision. */
  public static function seatedStock(scene:CollisionSceneBuilder, stock:Array<Int>, seats:Array<Int>):Void {
    for (part in stock) for (seat in seats) if (part != seat)
      scene.pair(scene.description.objects[part].name, scene.description.objects[seat].name,
        CollisionPairRule.Touch, CollisionPairStatus.ProcessContact, 0.0);
  }
}
