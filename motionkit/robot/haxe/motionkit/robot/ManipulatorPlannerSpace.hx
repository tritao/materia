package motionkit.robot;

import motionkit.planner.BoxClearanceSpace;
import robotkit.manipulation.Manipulator;

/** Maps the active manipulator coordinates into the complete robot's collision-planner state. */
class ManipulatorPlannerSpace implements BoxClearanceSpace {
  final inner:CollisionPlannerSpace;
  final arm:Manipulator;
  final reference:Array<Float>;
  final dofs:Array<Int>;
  final pointBuffer:Array<Float> = [];
  final mappedBuffer:Array<Float> = [];
  final widthBuffer:Array<Float> = [];

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
    var result = pointBuffer;
    result.resize(reference.length);
    for (i in 0...reference.length) result[i] = reference[i];
    for (i in 0...dofs.length) result[dofs[i]] = q[i];
    return result;
  }

  public function dimension():Int return dofs.length;
  public function lower():Array<Float> return [for (i in 0...dofs.length) arm.group.limitsOf(i).lower];
  public function upper():Array<Float> return [for (i in 0...dofs.length) arm.group.limitsOf(i).upper];
  public function invalid(q:Array<Float>):Null<String> return inner.invalid(full(q));
  public function violation(q:Array<Float>):Null<collisionkit.CollisionViolation> return inner.violation(full(q));

  public function edgesClear(edges:Array<Float>, count:Int):Array<Bool> {
    var n = dimension(), mapped = mappedBuffer;
    mapped.resize(0);
    if (edges == null || edges.length != count * n * 2) throw "Manipulator planner edges have the wrong dimension";
    for (edge in 0...count) for (endpoint in 0...2) {
      var offset = mapped.length;
      for (value in reference) mapped.push(value);
      for (i in 0...n) mapped[offset + dofs[i]] = edges[(edge * 2 + endpoint) * n + i];
    }
    return inner.edgesClear(mapped, count);
  }

  public function boxesClear(centres:Array<Float>, halfWidths:Array<Float>, count:Int):Array<Bool> {
    var n = dimension(), fullCentres = mappedBuffer, fullWidths = widthBuffer;
    fullCentres.resize(0); fullWidths.resize(0);
    if (centres == null || halfWidths == null || centres.length != count * n || halfWidths.length != count * n)
      throw "Manipulator planner boxes have the wrong dimension";
    for (box in 0...count) {
      var offset = fullCentres.length;
      for (value in reference) { fullCentres.push(value); fullWidths.push(0.0); }
      // Held coordinates do not move: only the manipulator's own coordinates span the box.
      for (i in 0...n) {
        fullCentres[offset + dofs[i]] = centres[box * n + i];
        fullWidths[offset + dofs[i]] = halfWidths[box * n + i];
      }
    }
    return inner.boxesClear(fullCentres, fullWidths, count);
  }

  public function dispose():Void inner.build.world.dispose();
}

