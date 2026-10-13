package motionkit.robot;

import motionkit.planner.PlannerSpace;
import robotkit.manipulation.Manipulator;

/** Maps the active manipulator coordinates into the complete robot's collision-planner state. */
class ManipulatorPlannerSpace implements PlannerSpace {
  final inner:CollisionPlannerSpace;
  final arm:Manipulator;
  final reference:Array<Float>;
  final dofs:Array<Int>;

  public function new(inner:CollisionPlannerSpace, arm:Manipulator, reference:Array<Float>, ?fullReference:Array<Float>) {
    this.inner = inner;
    this.arm = arm;
    this.reference = fullReference == null ? arm.stateOf(reference).q.copy() : fullReference.copy();
    if (this.reference.length != inner.bodies.model.dofCount())
      throw "Manipulator collision reference has the wrong complete-model dimension";
    var zero = [for (_ in 0...arm.dofCount()) 0.0];
    var base = arm.stateOf(zero).q;
    dofs = [];
    for (i in 0...zero.length) {
      var unit = zero.copy();
      unit[i] = 1.0;
      var changed = arm.stateOf(unit).q;
      var found = -1;
      for (j in 0...changed.length) if (Math.abs(changed[j] - base[j]) > 0.5) {
        if (found >= 0) throw "Manipulator coordinate maps to more than one collision DOF";
        found = j;
      }
      if (found < 0) throw 'Manipulator coordinate $i has no collision DOF';
      dofs.push(found);
    }
    inner.holdOtherDofs(this.reference, dofs);
  }

  function full(q:Array<Float>):Array<Float> {
    if (q == null || q.length != dofs.length) throw "Manipulator planner received the wrong manipulator dimension";
    var result = reference.copy();
    for (i in 0...dofs.length) result[dofs[i]] = q[i];
    return result;
  }

  public function dimension():Int return dofs.length;
  public function lower():Array<Float> return [for (i in 0...dofs.length) arm.group.limitsOf(i).lower];
  public function upper():Array<Float> return [for (i in 0...dofs.length) arm.group.limitsOf(i).upper];
  public function invalid(q:Array<Float>):Null<String> return inner.invalid(full(q));
  public function violation(q:Array<Float>):Null<collisionkit.CollisionViolation> return inner.violation(full(q));

  public function edgesClear(edges:Array<Float>, count:Int):Array<Bool> {
    var n = dimension(), mapped:Array<Float> = [];
    if (edges == null || edges.length != count * n * 2) throw "Manipulator planner edges have the wrong dimension";
    for (edge in 0...count) for (endpoint in 0...2) {
      var q = [for (i in 0...n) edges[(edge * 2 + endpoint) * n + i]];
      for (value in full(q)) mapped.push(value);
    }
    return inner.edgesClear(mapped, count);
  }

  public function dispose():Void inner.build.world.dispose();
}

