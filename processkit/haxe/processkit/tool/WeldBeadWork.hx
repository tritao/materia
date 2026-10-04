package processkit.tool;

import processkit.tool.GroundedWork.WeldBodyPose;
import robotkit.tool.ConvexSolid;

/** Deposited station triangles extruded along the seam; shared by sensing and clearance. */
class WeldBeadWork implements WeldWork {
  public final bead:WeldBead;
  final pose:Void -> WeldBodyPose;
  final sizes:Array<Float>;
  final solids:Array<Null<ConvexSolid>>;

  public function new(bead:WeldBead, pose:Void -> WeldBodyPose) {
    if (bead == null || pose == null) throw "Bead work needs a bead and its live frame";
    this.bead = bead;
    this.pose = pose;
    sizes = [for (_ in 0...bead.count) -1.0];
    solids = [for (_ in 0...bead.count) null];
  }

  /** The actual station prism in the bead frame, or no vertices before metal is deposited. */
  public function vertices(station:Int):Array<Float> {
    var leg = bead.leg(station);
    if (!(leg > 1e-8)) return [];
    var result:Array<Float> = [];
    for (distance in [bead.stationAt(station), bead.stationAt(station) + bead.binLength(station)]) {
      var base = bead.pointAt(distance);
      for (direction in [[0.0, 0.0, 0.0], bead.legA, bead.legB])
        for (axis in 0...3) result.push(base[axis] + direction[axis] * leg);
    }
    return result;
  }

  function solid(station:Int):Null<ConvexSolid> {
    var leg = bead.leg(station);
    if (sizes[station] != leg) {
      var points = vertices(station);
      solids[station] = points.length == 0 ? null : new ConvexSolid(points);
      sizes[station] = leg;
    }
    return solids[station];
  }

  public function distance(x:Float, y:Float, z:Float):Float {
    var local = localPoint(pose(), x, y, z);
    var along = progress(local);
    var nearest = Std.int(Math.max(0, Math.min(bead.count - 1, Math.floor(along / WeldBead.BIN))));
    var result = Math.POSITIVE_INFINITY;
    result = stationDistance(nearest, local, along, result);
    for (offset in 1...bead.count) {
      if (nearest - offset >= 0) result = stationDistance(nearest - offset, local, along, result);
      if (nearest + offset < bead.count) result = stationDistance(nearest + offset, local, along, result);
    }
    return result;
  }

  function progress(point:Array<Float>):Float {
    var result = 0.0;
    for (axis in 0...3) result += (point[axis] - bead.start[axis]) * bead.tangent[axis];
    return result;
  }

  function stationDistance(station:Int, point:Array<Float>, along:Float, best:Float):Float {
    var start = bead.stationAt(station), stop = start + bead.binLength(station);
    var bound = Math.max(0, Math.max(start - along, along - stop));
    if (bound > Math.max(0, best) + 1e-9) return best;
    var hull = solid(station);
    return hull == null ? best : Math.min(best, hull.distance(point[0], point[1], point[2]));
  }

  public function ray(ox:Float, oy:Float, oz:Float, dx:Float, dy:Float, dz:Float, range:Float):Float {
    var frame = pose();
    var origin = localPoint(frame, ox, oy, oz), direction = turnBack(frame.rotation, dx, dy, dz);
    var first = progress(origin), alongDirection = 0.0;
    for (axis in 0...3) alongDirection += direction[axis] * bead.tangent[axis];
    var last = first + alongDirection * range;
    var low = Math.min(first, last), high = Math.max(first, last);
    var result = Math.POSITIVE_INFINITY;
    for (station in 0...bead.count) {
      var start = bead.stationAt(station);
      if (start > high + 1e-9 || start + bead.binLength(station) < low - 1e-9) continue;
      var hull = solid(station);
      if (hull != null) result = Math.min(result, hull.ray(origin[0], origin[1], origin[2], direction[0], direction[1], direction[2], range));
    }
    return result;
  }

  static function localPoint(frame:WeldBodyPose, x:Float, y:Float, z:Float):Array<Float>
    return turnBack(frame.rotation, x - frame.position[0], y - frame.position[1], z - frame.position[2]);

  static function turnBack(q:Array<Float>, vx:Float, vy:Float, vz:Float):Array<Float> {
    var x = -q[0], y = -q[1], z = -q[2], w = q[3];
    var tx = 2 * (y * vz - z * vy), ty = 2 * (z * vx - x * vz), tz = 2 * (x * vy - y * vx);
    return [vx + w * tx + y * tz - z * ty, vy + w * ty + z * tx - x * tz, vz + w * tz + x * ty - y * tx];
  }
}
