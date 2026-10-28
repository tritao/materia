package machinekit.assembly;
import machinekit.assembly.AxisBuilder.AxisSpec;
import machinekit.component.MachineComponent;
import machinekit.transmission.*;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames;
import machinekit.assembly.Sense;
import machinekit.assembly.Sense.SenseTools;
/** Geometry for designing a support. This never installs a rack/drive. */
class RackMesh {
  static function dot(a:Array<Float>, b:Array<Float>):Float return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  static function cross(a:Array<Float>, b:Array<Float>):Array<Float> return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0]
  ];
  public static function backDepth(rack:MachineComponent):Float {
    if (Std.isOfType(rack, GearRackRun)) return cast(rack, GearRackRun).section.spec.pitchHeightMm;
    if (Std.isOfType(rack, GearRack)) return cast(rack, GearRack).spec.pitchHeightMm;
    if (Std.isOfType(rack, Rack)) {
      var r:Rack = cast rack;
      return 1.25 * r.moduleSize + r.barHeight;
    }
    if (Std.isOfType(rack, HelicalRack)) {
      var r:HelicalRack = cast rack;
      return 1.25 * r.n("normalModule") + r.n("barHeight");
    }
    throw "Rack recipe has no back datum";
  }
  public static function layout(
    axis:AxisSpec,
    direction:Array<Float>,
    pinion:MachineComponent,
    pose:AssemblyFrame,
    datum:AssemblyFrame,
    rack:MachineComponent,
    phaseOffsetMm:Float = 0
  ): {
    pose: AssemblyFrame,
    sense: Sense
  } {
    if (datum == null) throw "Rack mesh requires a declared back datum";
    machinekit.design.DesignGeometry.validateFrame(datum);
    var normal = AssemblyFrames.transformVector(datum, 0, 1, 0);
    var rackSide = [-normal.x, -normal.y, -normal.z];
    var wheel = RackGeometry.pinion(pinion), bar = RackGeometry.rack(rack);
    var a = AssemblyFrames.transformVector(pose, 0, 0, 1);
    var t = direction;
    var length = Math.sqrt(rackSide[0] * rackSide[0] + rackSide[1] * rackSide[1] + rackSide[2] * rackSide[2]);
    if (!(length > 0)) throw 'Rack of axis ${axis.id} needs a side';
    var n = [rackSide[0] / length, rackSide[1] / length, rackSide[2] / length];
    if (Math.abs(dot(n, [a.x, a.y, a.z])) > 1e-9 || Math.abs(dot(n,
      t)) > 1e-9) throw 'Rack of axis ${axis.id} must lie square to its pinion\'s axis and to the travel';
    // Rack frame: face along +X (the pinion's axis), teeth along +Y (towards the pinion), length along +Z = X x Y.
    var x = [a.x, a.y, a.z], y = [-n[0], -n[1], -n[2]];
    var z = cross(x, y);
    var radius = wheel.diameter / 2, pitch = Math.PI * wheel.moduleSize;
    // Pitch point at coordinate zero, on the pinion's face start; the face of the rack centred on the pinion's.
    var faceShift = (wheel.width - bar.width) / 2;
    var contact = [
      pose.x + radius * n[0] + faceShift * x[0],
      pose.y + radius * n[1] + faceShift * x[1],
      pose.z + radius * n[2] + faceShift * x[2]
    ];
    // The pinion's tooth nearest the pitch point: teeth are centred on multiples of 2 pi / teeth from its local +X.
    var local = AssemblyFrames.transformVector(AssemblyFrames.inverse(pose), n[0], n[1], n[2]);
    var angle = Math.atan2(local.y, local.x), step = 2 * Math.PI / wheel.teeth;
    var phase = wheel.hand * wheel.width * Math.tan(wheel.helix) / wheel.diameter;
    var offset = radius * (angle - phase - Math.round((angle - phase) / step) * step);
    // That tooth's centre lies `offset` behind the pitch point along the direction the pitch circle turns (a x n).
    var tangent = cross(x, n);
    var tooth = -offset * dot(tangent, z) - bar.hand * bar.width / 2 * Math.tan(bar.helix);
    // Rack gaps are centred on multiples of the pitch from its start: start the rack a whole number of pitches before
    // the tooth, as near as that comes to centring the rack on the pitch point's travel.
    var travel = dot(t, z);
    var low = Math.min(axis.lower * travel, axis.upper * travel), high = Math.max(
      axis.lower * travel,
      axis.upper * travel
    );
    var centred = (low + high) / 2 - bar.length / 2 + phaseOffsetMm * travel;
    var start = tooth - Math.round((tooth - centred) / pitch) * pitch;
    if (start > low - 2 * pitch || start + bar.length < high + 2
      * pitch) throw 'Rack of axis ${axis.id} is too short: it needs ${Math.ceil((high - low) / pitch) + 5} teeth, has ${bar.teeth}';
    var origin = [contact[0] + start * z[0], contact[1] + start * z[1], contact[2] + start * z[2]];
    // Rolling: the pinion's centre moves along a x n per unit turn of radius; it turns by (t . z) / radius per unit
    // travel, and Rack.relation gives alignment * 2 / pitch diameter.
    var sense = SenseTools.fromAlignment(travel > 0 ? 1 : -1);
    if (Math.abs(Math.abs(travel) - 1) > 1e-9) throw 'Rack of axis ${axis.id} must run along the travel';
    var result = {
      pose: AssemblyFrames.fromRotationMatrix(
        origin[0],
        origin[1],
        origin[2],
        [x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]]
      ),
      sense: sense
    };
    var up = AssemblyFrames.transformVector(datum, 0, 1, 0), along = AssemblyFrames.transformVector(datum, 0, 0, 1);
    if (Math.abs(Math.abs(dot(z,
      [along.x, along.y, along.z])) - 1) > 1e-9) throw "Rack datum tangent disagrees with mesh";
    var back = AssemblyFrames.transformPoint(result.pose, bar.width / 2, -backDepth(rack), bar.length / 2);
    if (Math.abs((back.x - datum.x) * up.x + (back.y - datum.y) * up.y + (back.z - datum.z) * up.z)
      > 1e-6) throw "Rack host back datum disagrees with pinion mesh";
    return result;
  }

}
