package machinekit.component;

import cadkit.parametric.HoleSpec;
import machinekit.component.ManufacturingRoute.RouteHole;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Between a hole table row (`RouteHole`) and the hole record a recording cuts (`HoleSpec`), in a part's frame. */
class RouteHoles {
  /**
   * `hole` as a hole record, `transform` taking the frame the row is given in to the part's own. The row's mouth and direction
   * become the hole's face frame (its x axis is any direction across the hole: a round hole does not depend on it).
   */
  public static function spec(hole:RouteHole, transform:AssemblyFrame):HoleSpec {
    var p = F.transformPoint(transform, hole.position[0], hole.position[1], hole.position[2]);
    var d = F.transformVector(transform, hole.direction[0], hole.direction[1], hole.direction[2]);
    var helper = Math.abs(d.z) < 0.9 ? {x: 0.0, y: 0.0, z: 1.0} : {x: 1.0, y: 0.0, z: 0.0};
    var dot = helper.x * d.x + helper.y * d.y + helper.z * d.z;
    var across = {x: helper.x - dot * d.x, y: helper.y - dot * d.y, z: helper.z - dot * d.z};
    if (hole.slotLength != null && hole.slotLength > 0) throw 'Hole ${hole.id} is a slot, which a hole record does not describe yet';
    return {frame: BoltedFace.frameOf(p, d, across), x: 0, y: 0, diameter: hole.diameter, depth: hole.depth, kind: hole.kind,
      thread: hole.thread, fit: hole.fit, nominal: hole.nominal, name: hole.id};
  }

  /** The table row for `spec` (its own frame), named `id`. */
  public static function route(spec:HoleSpec, id:String):RouteHole {
    var p = F.transformPoint(spec.frame, spec.x, spec.y, 0), d = F.transformVector(spec.frame, 0, 0, 1);
    return {id: id, kind: spec.kind ?? "clearance", thread: spec.thread, diameter: spec.diameter, nominal: spec.nominal ?? spec.diameter,
      fit: spec.fit, depth: spec.depth, position: [p.x, p.y, p.z], direction: [d.x, d.y, d.z]};
  }
}
