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
    height:Float,
    ?bridgeHeight:Float
  ):RackMount {
    var hostFrame = builder.zeroPose(host), shaft = F.transformVector(pinionPose, 0, 0, 1);
    var localShaft = F.transformVector(F.inverse(hostFrame), shaft.x, shaft.y, shaft.z);
    // The default bracket reaches a declared cross-section face, not an
    // inferred bounding-box side. A vertical column uses its other face:
    // the host's +Y may be parallel to the shaft instead of radial to it.
    var swapped = Math.abs(localShaft.y) > 1 - 1e-9;
    if (!swapped && Math.abs(localShaft.x)
      < 1 - 1e-9) throw "Nominal rack shaft must be normal to a declared host cross-section face";
    var crossSectionX = F.transformVector(hostFrame, swapped ? 0 : 1, swapped ? 1 : 0, 0);
    var up = F.transformVector(hostFrame, swapped ? 1 : 0, swapped ? 0 : 1, 0);
    var tangent = [
      crossSectionX.y * up.z - crossSectionX.z * up.y,
      crossSectionX.z * up.x - crossSectionX.x * up.z,
      crossSectionX.x * up.y - crossSectionX.y * up.x
    ];
    var frame = F.fromRotationMatrix(
      hostFrame.x,
      hostFrame.y,
      hostFrame.z,
      [crossSectionX.x, up.x, tangent[0], crossSectionX.y, up.y, tangent[1], crossSectionX.z, up.z, tangent[2]]
    );
    if (swapped) {
      var previousWidth = width;
      width = height;
      height = previousWidth;
    }
    var wheel = RackGeometry.pinion(pinion);
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
    if (swapped) {
      if (bridgeHeight == null || !Math.isFinite(bridgeHeight) || bridgeHeight <= height / 2 + 5
        || first.y - bridgeHeight <= 5) throw "Vertical rack requires a declared clear bridge below its back plane";
      // Two ordinary L members form a dogleg. The lower bridge clears the
      // carriage; the upper toe contacts the rack bottom beyond the motor
      // plate. Its return stays 2 mm outside the pinion's axial end face.
      var lowerWidth = outer + 7 - inner;
      var centre = F.transformPoint(frame, side * (inner + lowerWidth / 2), bridgeHeight, (first.z + second.z) / 2);
      var pose = F.fromRotationMatrix(
        centre.x,
        centre.y,
        centre.z,
        [cross.x, along[0], down.x, cross.y, along[1], down.y, cross.z, along[2], down.z]
      );
      var lower = new LinearMountSteel(new ComponentValues().setNumber("width", lowerWidth).setNumber(
        "length",
        bar.length
      ).setNumber(
        "thickness",
        5
      ).setNumber(
        "flangeWidth",
        bridgeHeight - height / 2 + 10
      ).setNumber(
        "flangeThickness",
        5
      ));
      builder.attach(id, lower, pose, host);
      var upperCentre = F.transformPoint(frame, side * (outer + 1 - bar.width / 2), first.y, (first.z + second.z) / 2);
      var upperPose = F.fromRotationMatrix(
        upperCentre.x,
        upperCentre.y,
        upperCentre.z,
        [-cross.x, -along[0], down.x, -cross.y, -along[1], down.y, -cross.z, -along[2], down.z]
      );
      var upper = new LinearMountSteel(new ComponentValues().setNumber("width", bar.width + 2).setNumber(
        "length",
        bar.length
      ).setNumber(
        "thickness",
        5
      ).setNumber(
        "flangeWidth",
        first.y - bridgeHeight
      ).setNumber(
        "flangeThickness",
        5
      ));
      builder.attach(id + "Return", upper, upperPose, id);
      return new RackMount(
        host,
        F.compose(F.inverse(hostFrame), datum),
        id + "Return",
        [{first: "$rack", second: id + "Return"}, {first: id + "Return", second: id}, {first: id, second: host}]
      );
    }
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
    return new RackMount(
      host,
      F.compose(F.inverse(hostFrame), datum),
      id,
      [{first: "$rack", second: id}, {first: id, second: host}]
    );
  }
}
