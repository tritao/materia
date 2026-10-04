package motionkit.robot;

import haxe.Int64;
import motionkit.Feed;
import motionkit.MotionOptions;
import motionkit.axis.MotionAxis;
import motionkit.path.PathPoint;
import motionkit.path.GeometricPath;
import motionkit.path.ArcSegment;
import motionkit.path.CornerBlender;
import motionkit.path.LineSegment;
import motionkit.path.PathPrimitiveKind;
import motionkit.path.QuinticBlend;
import motionkit.planner.JointPathSamples;
import motionkit.planner.PathTimingLimits;
import motionkit.planner.ToppraPathTiming;
import motionkit.planner.PathPlanningOptions;
import motionkit.trajectory.Trajectory;
import motionkit.trajectory.ValidationLimits;
import MotionKitNative;

typedef PathPlanRequest = {
  var path:GeometricPath;
  var pathOptions:Null<PathPlanningOptions>;
  var motionOptions:Null<MotionOptions>;
}

typedef LinearPathPlanRequest = {
  var target:PathPoint;
  var feed:Feed;
  var options:Null<MotionOptions>;
}

/** Cartesian planner with immutable machine configuration and no request state. */
class PathPlanner {
  final axes:Array<MotionAxis>;
  final fixedTimestepSeconds:Float;
  final modelRevision:Int64;
  final calibrationRevision:Int64;

  public function new(axes:Array<MotionAxis>, fixedTimestepSeconds:Float,
      modelRevision:Int64, calibrationRevision:Int64) {
    this.axes = axes.copy();
    this.fixedTimestepSeconds = fixedTimestepSeconds;
    this.modelRevision = modelRevision;
    this.calibrationRevision = calibrationRevision;
  }

  function axis(id:String):Null<MotionAxis> {
    for (value in axes) if (value.id == id) return value;
    return null;
  }

  public function planLinear(start:Array<Float>, request:LinearPathPlanRequest):PlanningResult {
    var target = request.target;
    var feed = request.feed;
    var options = request.options;
    if (target == null || feed == null) throw "Linear move needs a target and feed";
    var xAxis = requireAxis("x");
    var yAxis = requireAxis("y");
    var zAxis = requireAxis("z");
    var current = new PathPoint(xAxis.logicalPosition(start), yAxis.logicalPosition(start),
      zAxis.logicalPosition(start));
    var chosen = options == null ? new MotionOptions(feed.value, 0.0, 0.0) : options;
    var maxVelocity = chosen.maxVelocity <= 0.0
      ? feed.value : Math.min(feed.value, chosen.maxVelocity);
    var resolved = new MotionOptions(maxVelocity, chosen.maxAcceleration, chosen.maxJerk);
    return plan(start, {path: GeometricPath.lines([current, target]),
      pathOptions: PathPlanningOptions.exactStopMode(), motionOptions: resolved});
  }

  public function plan(start:Array<Float>, request:PathPlanRequest):PlanningResult {
    var path = request.path;
    var pathOptions = request.pathOptions;
    var motionOptions = request.motionOptions;
    var diagnostics:Array<String> = [];
    if (path == null) throw "Cartesian path is required";
    var options = pathOptions == null ? PathPlanningOptions.exactStopMode() : pathOptions;
    var planningPath = path;
    if (!options.exactStop && options.blendTolerance > 0.0) {
      var blended = CornerBlender.blend(path, options.blendTolerance * CornerBlender.GEOMETRY_SHARE,
        options.maxBlendTurnAngleRadians);
      planningPath = blended.path;
      diagnostics = blended.diagnostics;
    }
    var xAxis = requireAxis("x");
    var yAxis = requireAxis("y");
    var zAxis = requireAxis("z");
    var first = path.primitives[0].pointAt(0.0);
    var starts = [xAxis.logicalPosition(start), yAxis.logicalPosition(start),
      zAxis.logicalPosition(start)];
    var startCoordinates = [first.x, first.y, first.z];
    for (i in 0...3) {
      if (Math.abs(starts[i] - startCoordinates[i]) > 1e-8)
        throw 'Cartesian path starts at ${startCoordinates[i]} but axis ${["x", "y", "z"][i]} is at ${starts[i]}';
    }

    validatePathLimits(planningPath, [xAxis, yAxis, zAxis]);
    var maxVelocity = [for (_ in start) 1e8];
    var maxAcceleration = [for (_ in start) 1e8];
    var jointUnits = [for (_ in start) 1.0];
    var requested = motionOptions == null ? new MotionOptions() : motionOptions;
    for (direct in [xAxis, yAxis, zAxis]) {
      var unit = [for (_ in start) 0.0];
      direct.writeLogicalDelta(unit, 1.0);
      for (joint in direct.jointIndices) {
        var scale = Math.abs(unit[joint]);
        jointUnits[joint] = scale;
        maxVelocity[joint] = direct.maxVelocity * scale;
        maxAcceleration[joint] = direct.maxAcceleration * scale;
        if (requested.maxAcceleration > 0.0)
          maxAcceleration[joint] = Math.min(maxAcceleration[joint],
            requested.maxAcceleration * scale);
      }
    }
    var segments:Array<{timeFromStartNs:Int64, durationNs:Int64,
      coefficients:Array<Array<Float>>}> = [];
    var junctionSpeeds = [for (_ in 0...(planningPath.primitives.length + 1)) 0.0];
    if (!options.exactStop) for (index in 1...planningPath.primitives.length) {
      var before = planningPath.primitives[index - 1];
      var after = planningPath.primitives[index];
      if (before.kind() == PathPrimitiveKind.Circular ||
          after.kind() == PathPrimitiveKind.Circular) continue;
      var beforeTangent = before.tangentAt(before.length());
      var afterTangent = after.tangentAt(0.0);
      var tangentDot = 0.0;
      for (coordinate in 0...3)
        tangentDot += beforeTangent[coordinate] * afterTangent[coordinate];
      if (tangentDot < 0.99999) continue;
      var speed = requested.maxVelocity > 0.0 ? requested.maxVelocity : 1e8;
      var pathAcceleration = 1e8;
      for (side in [before, after]) {
        for (sample in 0...33) {
          var distance = side.length() * sample / 32.0;
          var tangent = side.tangentAt(distance);
          var curvature = side.curvatureAt(distance);
          var prime = [for (_ in start) 0.0];
          var second = [for (_ in start) 0.0];
          for (entry in [{axis: xAxis, prime: tangent[0], second: -tangent[1] * curvature},
              {axis: yAxis, prime: tangent[1], second: tangent[0] * curvature},
              {axis: zAxis, prime: tangent[2], second: 0.0}]) {
            entry.axis.writeLogicalDelta(prime, entry.prime);
            entry.axis.writeLogicalDelta(second, entry.second);
          }
          for (joint in 0...start.length) {
            if (Math.abs(prime[joint]) > 1e-12) {
              speed = Math.min(speed, maxVelocity[joint] / Math.abs(prime[joint]));
              pathAcceleration = Math.min(pathAcceleration,
                maxAcceleration[joint] / Math.abs(prime[joint]));
            }
            if (Math.abs(second[joint]) > 1e-12)
              speed = Math.min(speed,
                Math.sqrt((Std.isOfType(side, QuinticBlend) ? 0.5 : 1.0) *
                  maxAcceleration[joint] / Math.abs(second[joint])));
          }
        }
      }
      speed = Math.min(speed * 0.75,
        0.75 * Math.sqrt(2.0 * pathAcceleration *
          Math.min(before.length(), after.length())));
      junctionSpeeds[index] = speed;
    }
    var offset = Int64.ofInt(0);
    var previousEnd:Null<PathPoint> = null;
    var worstTaskDeviation = 0.0;
    var worstTaskTime = 0.0;
    for (primitiveIndex in 0...planningPath.primitives.length) {
      var primitive = planningPath.primitives[primitiveIndex];
      var length = primitive.length();
      var primitiveStart = primitive.pointAt(0.0);
      if (previousEnd != null && previousEnd.distanceTo(primitiveStart) > 1e-8)
        throw "Path primitives must form a connected path";
      previousEnd = primitive.pointAt(length);
      if (length <= 1e-12) continue;
      var count = 1;
      if (primitive.kind() == PathPrimitiveKind.Arc) {
        var arc:ArcSegment = cast primitive;
        count = Std.int(Math.ceil(Math.abs(arc.sweepAngle) * 16.0));
      }
      if (primitive.kind() == PathPrimitiveKind.Circular) {
        var circular:motionkit.path.CircularSegment = cast primitive;
        count = Std.int(Math.ceil(Math.abs(circular.sweepAngle) * 16.0));
      }
      if (primitive.kind() == PathPrimitiveKind.Blend) count = 32;
      var distances:Array<Float> = [];
      var positions:Array<Array<Float>> = [];
      var first:Array<Array<Float>> = [];
      var second:Array<Array<Float>> = [];
      for (index in 0...(count + 1)) {
        var distance = index == count ? length : length * index / count;
        var point = primitive.pointAt(distance);
        var tangent = primitive.tangentAt(distance);
        var curvature = primitive.curvatureAt(distance);
        var secondDerivative = primitive.kind() == PathPrimitiveKind.Circular ?
          (cast primitive:motionkit.path.CircularSegment).secondDerivativeAt(distance) :
          [-tangent[1] * curvature, tangent[0] * curvature, 0.0];
        var q = start.copy();
        var qPrime = [for (_ in start) 0.0];
        var qDoublePrime = [for (_ in start) 0.0];
        for (entry in [{axis: xAxis, position: point.x, prime: tangent[0],
            second: secondDerivative[0]},
            {axis: yAxis, position: point.y, prime: tangent[1],
              second: secondDerivative[1]},
            {axis: zAxis, position: point.z, prime: tangent[2],
              second: secondDerivative[2]}]) {
          entry.axis.writeLogicalPosition(q, entry.position);
          entry.axis.writeLogicalDelta(qPrime, entry.prime);
          entry.axis.writeLogicalDelta(qDoublePrime, entry.second);
        }
        distances.push(distance);
        // The lowering tolerance is a Cartesian distance. Retiming and
        // Hermite fitting use equivalent carriage units for every shaft too.
        positions.push([for (joint in 0...start.length) q[joint] / jointUnits[joint]]);
        first.push([for (joint in 0...start.length) qPrime[joint] / jointUnits[joint]]);
        second.push([for (joint in 0...start.length) qDoublePrime[joint] / jointUnits[joint]]);
      }
      var jointPath = new JointPathSamples(distances, positions, first, second);
      var speedCaps = requested.maxVelocity > 0.0
        ? [for (_ in 0...count) requested.maxVelocity] : [];
      // Leave room for nanosecond stage rounding and Hermite coefficient
      // roundoff before the runtime validates exact polynomial extrema.
      var limits = new PathTimingLimits(
        [for (joint in 0...start.length) maxVelocity[joint] / jointUnits[joint] * 0.999],
        [for (joint in 0...start.length) maxAcceleration[joint] / jointUnits[joint] * 0.999], speedCaps,
        junctionSpeeds[primitiveIndex], junctionSpeeds[primitiveIndex + 1]);
      var loweringTolerance = options.exactStop ? 1e-6 :
        Math.min(1e-6, options.blendTolerance * 0.01);
      var timed = new ToppraPathTiming(loweringTolerance).time(jointPath, limits);
      var pieceDuration = timed.trajectory.durationSeconds();
      var sampleCount = Std.int(Math.ceil(pieceDuration / 0.001));
      for (sampleIndex in 0...(sampleCount + 1)) {
        var localTime = pieceDuration * sampleIndex / sampleCount;
        var normalizedActual = timed.trajectory.evaluate(localTime).positions;
        var actual = [for (joint in 0...start.length) normalizedActual[joint] * jointUnits[joint]];
        var tool = new PathPoint(xAxis.logicalPosition(actual),
          yAxis.logicalPosition(actual), zAxis.logicalPosition(actual));
        var deviation = distanceToAuthoredPath(tool, path);
        if (deviation > worstTaskDeviation) {
          worstTaskDeviation = deviation;
          worstTaskTime = Int64.toFloat(offset) * 1e-9 + localTime;
        }
      }
      var pieceSegments = timed.trajectory.segments();
      for (segment in pieceSegments) {
        // Fit each independent coordinate once. Followers come from the axis mapping,
        // rather than independent Hermite fits whose large coefficients can drift apart.
        var coefficients = [for (joint in 0...start.length)
          [for (value in segment.coefficients[joint]) value * jointUnits[joint]]];
        for (power in 0...coefficients[0].length) {
          var values = [for (joint in 0...start.length) coefficients[joint][power]];
          for (direct in [xAxis, yAxis, zAxis]) {
            var logical = power == 0 ? direct.logicalPosition(values) :
              values[direct.jointIndices[0]] / direct.jointScale(0);
            if (power == 0) direct.writeLogicalPosition(values, logical);
            else direct.writeLogicalDelta(values, logical);
          }
          for (joint in 0...start.length) coefficients[joint][power] = values[joint];
        }
        segments.push({timeFromStartNs: Int64.add(offset, segment.timeFromStartNs),
          durationNs: segment.durationNs, coefficients: coefficients});
      }
      var last = pieceSegments[pieceSegments.length - 1];
      offset = Int64.add(offset, Int64.add(last.timeFromStartNs, last.durationNs));
      timed.releaseDistanceMap();
      timed.trajectory.dispose();
    }
    if (segments.length == 0)
      return {trajectory: Trajectory.fromPositionSamples(
        [0.0, fixedTimestepSeconds], [start, start]),
        report: null, diagnostics: diagnostics};
    var finalPoint = previousEnd;
    if (finalPoint == null) throw "Cartesian path has no endpoint";
    var finalPosition = start.copy();
    xAxis.writeLogicalPosition(finalPosition, finalPoint.x);
    yAxis.writeLogicalPosition(finalPosition, finalPoint.y);
    zAxis.writeLogicalPosition(finalPosition, finalPoint.z);
    segments.push({timeFromStartNs: offset, durationNs: Int64.ofInt(1),
      coefficients: [for (position in finalPosition) [position]]});
    var result = Trajectory.fromSegments(segments);
    var validation = new ValidationLimits(start.length, modelRevision, calibrationRevision);
    validation.continuity(0, 1e-9);
    for (joint in 0...start.length) {
      validation.velocity(joint, maxVelocity[joint] / jointUnits[joint]);
      validation.acceleration(joint, maxAcceleration[joint] / jointUnits[joint]);
    }
    for (direct in [xAxis, yAxis, zAxis]) {
      var lower = start.copy();
      var upper = start.copy();
      direct.writeLogicalPosition(lower, direct.lowerLimit);
      direct.writeLogicalPosition(upper, direct.upperLimit);
      for (joint in direct.jointIndices)
        validation.position(joint, Math.min(lower[joint], upper[joint]) / jointUnits[joint],
          Math.max(lower[joint], upper[joint]) / jointUnits[joint]);
    }
    // Cartesian claims are in metres. Validate every mapped shaft in equivalent
    // carriage units, so a screw ratio does not turn a 1 nm seam claim into radians.
    var normalized = Trajectory.fromSegments([for (segment in segments) {
      timeFromStartNs: segment.timeFromStartNs, durationNs: segment.durationNs,
      coefficients: [for (joint in 0...start.length)
        [for (value in segment.coefficients[joint]) value / jointUnits[joint]]]
    }]);
    var report = normalized.validate(validation);
    normalized.dispose();
    var tolerance = options.exactStop || options.blendTolerance == 0.0
      ? 1e-5 : options.blendTolerance;
    report.setTaskSpace(worstTaskDeviation <= tolerance
      ? MotionKitNativeConstants.MK_CHECK_PASSED
      : MotionKitNativeConstants.MK_CHECK_FAILED,
      worstTaskDeviation, worstTaskTime, tolerance, Int64.ofInt(1000000));
    if (report.hasFailure()) {
      for (index in 0...report.checks.length) {
        var check = report.checks[index];
        if (check.status == MotionKitNativeConstants.MK_CHECK_FAILED)
          throw 'Timed Cartesian path check $index failed: ${check.value} > ${check.limit} at ${check.timeSeconds}';
      }
    }
    result.controlAcceleration = maxAcceleration.copy();
    return {trajectory: result, report: report, diagnostics: diagnostics};
  }

  function distanceToAuthoredPath(point:PathPoint, path:GeometricPath):Float {
    var closest = Math.POSITIVE_INFINITY;
    for (primitive in path.primitives) {
      if (primitive.kind() == PathPrimitiveKind.Line) {
        var line:LineSegment = cast primitive;
        var length = line.length();
        if (length <= 0.0) {
          closest = Math.min(closest, point.distanceTo(line.start));
          continue;
        }
        var direction = line.tangentAt(0.0);
        var projection = (point.x - line.start.x) * direction[0] +
          (point.y - line.start.y) * direction[1] +
          (point.z - line.start.z) * direction[2];
        closest = Math.min(closest, point.distanceTo(
          line.pointAt(Math.max(0.0, Math.min(length, projection)))));
      } else if (primitive.kind() == PathPrimitiveKind.Arc) {
        var arc:ArcSegment = cast primitive;
        var angle = Math.atan2(point.y - arc.center.y, point.x - arc.center.x);
        var baseShift = Math.round((arc.startAngle - angle) / (2.0 * Math.PI));
        for (shift in -2...3) {
          var candidate = angle + 2.0 * Math.PI * (baseShift + shift);
          var fraction = arc.sweepAngle == 0.0 ? 0.0 :
            (candidate - arc.startAngle) / arc.sweepAngle;
          var distance = arc.length() * Math.max(0.0, Math.min(1.0, fraction));
          closest = Math.min(closest, point.distanceTo(arc.pointAt(distance)));
        }
      } else if (primitive.kind() == PathPrimitiveKind.Circular) {
        var circular:motionkit.path.CircularSegment = cast primitive;
        closest = Math.min(closest, circular.distanceTo(point));
      } else {
        for (sample in 0...129)
          closest = Math.min(closest,
            point.distanceTo(primitive.pointAt(primitive.length() * sample / 128.0)));
      }
    }
    return closest;
  }

  function validatePathLimits(path:GeometricPath, directAxes:Array<MotionAxis>):Void {
    for (primitive in path.primitives) {
      var length = primitive.length();
      for (sampleIndex in 0...65) {
        var point = primitive.pointAt(length * sampleIndex / 64.0);
        var coordinates = [point.x, point.y, point.z];
        for (i in 0...3) {
          var roundoff = 1e-12 * Math.max(1.0, directAxes[i].upperLimit - directAxes[i].lowerLimit);
          if (coordinates[i] < directAxes[i].lowerLimit - roundoff ||
              coordinates[i] > directAxes[i].upperLimit + roundoff)
            throw 'Axis "${["x", "y", "z"][i]}" path point ${coordinates[i]} is outside its limits';
        }
      }
    }
  }

  function requireAxis(id:String):MotionAxis {
    var result = axis(id);
    if (result == null) throw 'Motion system needs a "$id" axis';
    return cast result;
  }
}
