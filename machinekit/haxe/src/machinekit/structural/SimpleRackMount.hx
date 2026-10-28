package machinekit.structural;

import machinekit.assembly.*;
import machinekit.assembly.AxisBuilder.AxisSpec;
import machinekit.component.*;
import machinekit.transmission.RackGeometry;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;

/** Continuous steel back bracket for nominal/example racks without catalogue
 * drilling. The bracket physically reaches the explicitly dimensioned host side.
 * This does not assign invented supplier holes, ratings or RM2 qualification. */
class SimpleRackMount {
  public static function build(
    builder:AxisBuilder,
    id:String,
    axis:AxisSpec,
    direction:Array<Float>,
    pinion:MachineComponent,
    pinionPose:AssemblyFrame,
    rack:MachineComponent,
    host:String,
    width:Float,
    height:Float
  ):RackMount {
    var frame = builder.zeroPose(host), up = F.transformVector(frame, 0, 1, 0), wheel = RackGeometry.pinion(pinion);
    var depth = RackMesh.backDepth(rack), offset = wheel.diameter / 2 + depth;
    var datum = AxisBuilder.orient(
      pinionPose.x - up.x * offset,
      pinionPose.y - up.y * offset,
      pinionPose.z - up.z * offset,
      [up.x, up.y, up.z],
      direction
    );
    var mesh = RackMesh.layout(axis, direction, pinion, pinionPose, datum, rack).pose, bar = RackGeometry.rack(rack);
    var inverse = F.inverse(frame);
    var first = F.transformPoint(F.compose(inverse, mesh), 0, -depth, 0);
    var second = F.transformPoint(F.compose(inverse, mesh), bar.width, -depth, bar.length);
    var side = (first.x + second.x) / 2 < 0 ? -1.0 : 1.0;
    var outer = Math.max(side * first.x, side * second.x), inner = width / 2 + 5;
    if (outer <= inner) throw "Nominal rack bracket cannot fit beside its declared host";
    var span = outer - inner, drop = Math.max(6, first.y - height / 2 + 10);
    var cross = F.transformVector(frame, side, 0, 0), down = F.transformVector(frame, 0, -1, 0);
    var along = [
      down.y * cross.z - down.z * cross.y,
      down.z * cross.x - down.x * cross.z,
      down.x * cross.y - down.y * cross.x
    ];
    var centre = F.transformPoint(frame, side * (inner + span / 2), first.y, (first.z + second.z) / 2);
    var pose = F.fromRotationMatrix(
      centre.x,
      centre.y,
      centre.z,
      [cross.x, along[0], down.x, cross.y, along[1], down.y, cross.z, along[2], down.z]
    );
    var stock = new LinearMountSteel(new ComponentValues().setNumber("width",
      span).setNumber("length", bar.length).setNumber(
        "thickness",
        5
      ).setNumber(
        "flangeWidth",
        drop
      ).setNumber(
        "flangeThickness",
        5
      ));
    builder.attach(id, stock, pose, host);
    return new RackMount(host,
      F.compose(inverse, datum), id, [{first: "$rack", second: id}, {first: id, second: host}]);
  }
}
