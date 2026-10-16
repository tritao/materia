package motionkit.robot;

import motionkit.path.PosePath;
import motionkit.path.PosePrimitive;

/** A joint-space sample of a path: where, on which primitive, and the primitive arriving there. */
typedef PathSample = {
  var distance:Float;
  var primitive:PosePrimitive;
  var local:Float;
  var arriving:Null<{primitive:PosePrimitive, local:Float}>;
}

/** Geometry-only sampling before inverse kinematics and timing. No compiler or solver state is retained. */
class PathSampler {
  /**
    Most samples one timed section holds. An arm's samples each cost an inverse-kinematics search, so its budget is
    small; on a machine whose joints are an exact affine map a straight piece is one sample, and what a section costs is
    native memory (a sample is 2 KB).
  **/
  public static inline final MAX_SECTION_SAMPLES = 10000;
  public static inline final MAX_AFFINE_SECTION_SAMPLES = 100000;

  /** Check the number of spans, excluding the initial sample, against the solver's sampling budget. */
  public static function requireBudget(spans:Int, affine:Bool, opIndex:Int):Void {
    if (spans > (affine ? MAX_AFFINE_SECTION_SAMPLES : MAX_SECTION_SAMPLES))
      throw 'Motion program op $opIndex exceeds Cartesian sample budget';
  }

  /**
    `path` split at its sharp corners, where one primitive's end tangent is not the next one's start tangent (authored
    task-velocity discontinuities, each timed from rest to rest), with each section's distance along it.
  **/
  public static function timingSections(path:PosePath):Array<{path:PosePath, offset:Float}> {
    var result:Array<{path:PosePath, offset:Float}> = [];
    var cursor = new TimingSectionCursor(path);
    while (cursor.hasNext()) {
      var section=cursor.next();result.push({path:section.path,offset:section.offset});
    }
    return result;
  }

  /**
    Where to sample a path in joint space: every primitive boundary, so that
    each span lies on one primitive, and inside each primitive at most
    `cartesianResolution` apart and a quarter radian of turning apart. A
    boundary sample also names the primitive arriving at it.
  **/
  public static function samples(path:PosePath, cartesianResolution:Float, angularResolution:Float,
      exact:Bool):Array<PathSample> {
    var first = path.primitives[0];
    // On a machine whose joints are an exact affine map of the pose a straight piece needs no samples inside it.
    var samples:Array<PathSample> = [{distance: 0.0, primitive: first, local: 0.0, arriving: null}];
    var start = 0.0;
    // A distance-driven joint changes its rate's slope at its stations: they are sampled too.
    var stations = path.drive == null ? [] : path.drive.stations();
    var stationAt = 0;
    for (k in 0...path.primitives.length) {
      var primitive = path.primitives[k], length = primitive.length();
      var pieces = motionkit.path.PoseSampling.pieces(primitive,cartesianResolution,angularResolution,exact);
      var locals = [for (piece in 1...pieces) length * piece / pieces];
      while (stationAt < stations.length && stations[stationAt] <= start + 1e-12) stationAt++;
      var extra = stationAt;
      while (extra < stations.length && stations[extra] < start + length - 1e-12) { locals.push(stations[extra] - start); extra++; }
      if (extra > stationAt) locals.sort(Reflect.compare);
      // Exactly the end: `length * pieces / pieces` can round one ulp past it.
      locals.push(length);
      var previous = 0.0;
      for (index in 0...locals.length) {
        var local = locals[index], last = index == locals.length - 1;
        if (!last && local - previous <= 1e-12) continue;
        previous = local;
        var next = last && k + 1 < path.primitives.length ? path.primitives[k + 1] : null;
        samples.push(next == null ?
          {distance: start + local, primitive: primitive, local: local, arriving: null} :
          {distance: start + length, primitive: next, local: 0.0,
            arriving: {primitive: primitive, local: length}});
      }
      start += length;
    }
    samples[samples.length - 1].distance = path.length();
    return samples;
  }
}

/** Lazily constructs only the current exact-stop section and its local drive. */
class TimingSectionCursor {
  final source:PosePath;
  var first:Int=0;
  public function new(source:PosePath) this.source=source;
  public function hasNext():Bool return first<source.primitives.length;
  public function next():{path:PosePath,offset:Float,last:Bool} {
    if (!hasNext()) throw "Timing section cursor is finished";
    var begin=first,end=source.primitives.length;
    for (k in (begin+1)...source.primitives.length) {
      var before=source.primitives[k-1],after=source.primitives[k];
      var arriving=before.derivativesAt(before.length()),leaving=after.derivativesAt(0.0);
      var beforeAngular=ToolFreedom.requiredAngular(before.endWaypoint().pose,arriving.angular,before.orientationPolicy());
      var afterAngular=ToolFreedom.requiredAngular(after.startWaypoint().pose,leaving.angular,after.orientationPolicy());
      var turn=0.0;
      for (axis in 0...3) turn=Math.max(turn,Math.max(Math.abs(arriving.linear[axis]-leaving.linear[axis]),
        Math.abs(beforeAngular[axis]-afterAngular[axis])));
      if (turn>1e-6) {end=k;break;}
    }
    first=end;
    return {path:begin==0 && end==source.primitives.length ? source : source.sub(begin,end),
      offset:source.startOf(begin),last:end==source.primitives.length};
  }
}
