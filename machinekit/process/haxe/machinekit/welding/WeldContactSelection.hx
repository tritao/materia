package machinekit.welding;

import cadkit.modeling.Vector;
import machinekit.welding.WeldProbeGeometry;
import machinekit.welding.WeldProbeGeometry.WeldProbeFace;
import machinekit.welding.WeldProbePatterns;
import materia.project.SceneContactRegistration.SceneContactWork;
import machinekit.welding.WeldProbeParkingBounds.WeldProbeRegionBounds;
import machinekit.welding.WeldProbeUncertainty.WeldProbeObservedBounds;
import processkit.ProbeMotionPlanner;
import processkit.ProbePosePlanner;
import processkit.ContactProbeRunner.ContactProbeRequest;
import processkit.ContactProbeRunner;
import processkit.perception.ContactRegistrationPrecision;
import processkit.perception.ContactPoseEnvelope;
import processkit.perception.ContactRegistrationSequence;
import processkit.perception.ContactRegistrationSequence.ContactRegistrationStage;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import haxe.io.BytesOutput;

/** Bridge saved nominal CAD patches to measured registration and checked arm probe preparation. */
class WeldContactSelection {
  /** Small reserve above the standard 3 mm air-clearance minimum. */
  static inline final PROBE_AIR_STANDOFF = 0.0032;
  static inline final PROBE_PATCH_CLEARANCE_MM = 3.0;
  final job:SceneContactWork;
  final geometry:WeldProbeGeometry;
  final prepare:ProbePosePlanner;
  final positions:Void->Array<Float>;
  final precision:ContactRegistrationPrecision;
  final contactToolPrefix:Null<String>;
  public function new(job:SceneContactWork, planner:ProbeMotionPlanner, positions:Void->Array<Float>,
      ?contactToolOccurrence:String, ?contactToolMember:String) {
    if (job == null || planner == null || positions == null) throw "Contact selection needs a saved CAD job and observed arm joints";
    var toolPrefix:Null<String> = contactToolMember;
    if (toolPrefix != null && toolPrefix.length == 0)
      throw "Contact selection needs a non-empty exact CAD tool member";
    if (toolPrefix == null && contactToolOccurrence != null) {
      var separator = contactToolOccurrence.lastIndexOf("/");
      if (separator <= 0 || separator >= contactToolOccurrence.length - 1)
        throw "Contact selection needs a tool connector occurrence below its collision body";
      toolPrefix = contactToolOccurrence.substr(0, separator);
    }
    this.job = job; this.positions = positions; contactToolPrefix = toolPrefix;
    geometry = WeldProbeGeometry.fromContactFaces(job.faces, 0.001);
    precision = new ContactRegistrationPrecision([for (face in geometry.faces) if (face.target)
      for (edge in face.chords) for (point in edge) metres(point)]);
    prepare = new ProbePosePlanner(planner);
  }
  static function metres(point:Vector):Vec3 return new Vec3(point.x * 0.001, point.y * 0.001, point.z * 0.001);
  static function direction(point:Vector):Vec3 return new Vec3(point.x, point.y, point.z);

  public function sequence(nominalRootWork:Transform3):ContactRegistrationSequence {
    var envelope = new ContactPoseEnvelope(nominalRootWork, Vec3.fromArray(job.translation),
      Vec3.fromArray(job.rotation), job.measurementError);
    var savedStart = positions().copy();
    return new ContactRegistrationSequence(envelope,
      (count, normals, current) -> selectFromEntries(count, normals, current, savedStart));
  }

  function screen(estimate:Transform3, start:Array<Float>, discover:Bool,
      lifts:Map<String, Float>):WeldProbeFace->Vector->WeldProbeRegionBounds->Bool {
    if (discover) {
      // Screen nearby points with one inexpensive free-roll branch. Full
      // branch search and its motion/corridor proof belong to selected probes.
      return (face, point, bounds) -> {
        var lift = prepare.checkedApproachLift(estimate.transformPoint(metres(point)),
          estimate.rotation.rotate(direction(face.normal)), bounds.normalTravel * 0.001, start, 0.04,
          PROBE_AIR_STANDOFF);
        if (lift == null) return false;
        lifts.set(pointKey(geometry.faces.indexOf(face), point), cast lift);
        return true;
      };
    }
    return (face, point, bounds) -> {
      var lift = prepare.observedApproachLift(estimate.transformPoint(metres(point)),
        estimate.rotation.rotate(direction(face.normal)), bounds.normalTravel * 0.001, start, 0.01,
        PROBE_AIR_STANDOFF);
      if (lift == null) return false;
      lifts.set(pointKey(geometry.faces.indexOf(face), point), cast lift);
      return true;
    };
  }

  function bounds(envelope:ContactPoseEnvelope, estimate:Transform3):WeldProbeUncertainty {
    var regions = new Map<String, WeldProbeRegionBounds>();
    return new WeldProbeObservedBounds((face, point) -> {
      var key = pointKey(geometry.faces.indexOf(face), point);
      var cached = regions.get(key);
      if (cached != null) return cached;
      var observed = envelope.region(metres(point), direction(face.normal), direction(face.u), direction(face.v), estimate);
      var result = new WeldProbeRegionBounds(observed.halfU * 1000, observed.halfV * 1000,
        observed.normalTravel * 1000, observed.tiltU, observed.tiltV);
      regions.set(key, result);
      return result;
    });
  }

  /** Plan from the measured terminal posture first, then a checked return to the saved start. */
  function selectFromEntries(count:Int, normals:Array<Vec3>, envelope:ContactPoseEnvelope,
      savedStart:Array<Float>):ContactRegistrationStage {
    var observed = positions();
    var entries = [observed];
    if (!sameJoints(observed, savedStart)) entries.push(savedStart.copy());
    var reasons:Array<String> = [];
    for (entry in entries) {
      try {
        if (!sameJoints(entry, observed)) prepare.motion.approachJoints(entry, observed);
        var stage = select(count, normals, envelope, entry);
        return new ContactRegistrationStage(stage.normal, stage.offset, stage.probes, entry);
      } catch (error:Dynamic) reasons.push(Std.string(error));
    }
    throw 'No checked $count-contact stage from the measured posture or saved start: ${reasons.join("; ")}';
  }

  static function sameJoints(a:Array<Float>, b:Array<Float>):Bool {
    if (a == null || b == null || a.length != b.length) return false;
    for (i in 0...a.length) if (Math.abs(a[i] - b[i]) > 1e-9) return false;
    return true;
  }

  function select(count:Int, normals:Array<Vec3>, envelope:ContactPoseEnvelope,
      start:Array<Float>):ContactRegistrationStage {
    var estimate = envelope.estimate();
    var toolRadiusMm = contactToolPrefix == null ? 0.0 : processkit.tool.ToolContactProfile.FRONT_RADIUS_METERS * 1000.0;
    // The envelope cannot change during this synchronous selection. Reuse exact
    // face/point bounds across ranking, reach screens and lattice refinement.
    var provider = bounds(envelope, estimate);
    var reasons:Array<String> = [];
    var reported = new Map<String, Bool>();
    // At each resolution, try observed continuation before global discovery.
    // Geometric ranking is cheap; exact preparation proves each selected probe.
    for (divisions in [9, 17, 33]) for (discover in [false, true]) {
      var previous = [for (normal in normals) new Vector(normal.x, normal.y, normal.z)];
      var screenedLifts = new Map<String, Float>();
      var possible = count == 2 ? screen(estimate, start, discover, screenedLifts) : null;
      var geometric = WeldProbePatterns.stages(geometry, count, previous, provider,
        PROBE_PATCH_CLEARANCE_MM, divisions, possible, null, toolRadiusMm);
      // Rank by geometric observability, then prove one face at a time.
      for (candidate in geometric) {
        var stages:Array<WeldProbeStage> = [candidate];
        if (count == 3) {
          // The same observable triangle can leave the arm in very different
          // terminal postures depending on contact order. Try alternate orders
          // before moving on to another CAD face.
          var points = candidate.points;
          var orders:Array<Array<Int>> = [[0, 2, 1], [1, 2, 0], [2, 1, 0]];
          for (order in orders) stages.push(new WeldProbeStage(candidate.face,
            [points[order[0]], points[order[1]], points[order[2]]], candidate.approach, candidate.score));
        }
        if (count == 1) {
          // Prove a bounded number of screen-positive points first. A failed screen is
          // only a ranking signal, so retain the geometric centre as a full-proof fallback.
          var screened = WeldProbePatterns.stages(geometry, count, previous, provider, PROBE_PATCH_CLEARANCE_MM, divisions,
            screen(estimate, start, discover, screenedLifts), candidate.face, toolRadiusMm);
          // The endpoint screen only ranks candidates; a checked route can still
          // reject the closest points. Try the full finite spread (at most eight
          // samples per face) before declaring that this plane has no reachable
          // contact, especially after live measurements shift the pose estimate.
          stages = [for (stage in screened) if (stage.screenPassed) stage];
          var geometricCovered = false;
          for (stage in stages) if (samePattern(stage, candidate)) geometricCovered = true;
          if (!geometricCovered) stages.push(candidate);
        }
        for (stage in stages) {
          try {
            var probes:Array<ContactProbeRequest> = [];
            var normal = direction(stage.face.normal);
            var error = precision.contactError([for (point in stage.points) metres(point)], normal,
              normals, job.measurementError);
            var from = start;
            for (point in stage.points) {
              var bounds = provider.region(stage.face, point);
              var rootPoint = estimate.transformPoint(metres(point));
              var rootNormal = estimate.rotation.rotate(normal);
              var screenedLift = screenedLifts.get(pointKey(geometry.faces.indexOf(stage.face), point));
              var initialLift = screenedLift == null ? 0.0 : cast screenedLift;
              var checked = discover
                ? prepare.prepare(rootPoint, rootNormal, bounds.normalTravel * 0.001, from, job.contactOffset, PROBE_AIR_STANDOFF, 16, initialLift)
                : prepare.prepareObserved(rootPoint, rootNormal, bounds.normalTravel * 0.001, from, job.contactOffset, PROBE_AIR_STANDOFF, 16, initialLift);
              var denominator = rootNormal.dot(checked.direction);
              if (Math.abs(denominator) < 1e-8) throw "Prepared sensing ray is tangent to its selected CAD plane";
              var planeTravel = rootNormal.dot(rootPoint.sub(checked.approach.translation)) / denominator;
              if (!Math.isFinite(planeTravel) || planeTravel <= 0 || planeTravel > checked.distance + 1e-8)
                throw "Selected CAD contact plane lies outside its checked sensing corridor";
              // A displaced surface can trigger touch before the nominal CAD plane.
              // Prove static clearance only up to the earliest possible touch;
              // measured sensing and its checked braking path own the remaining
              // interval. Stop the static sweep outside the tool's contact margin;
              // the runtime opens the named target window when that margin is reached.
              var checkedTravel = checked.precontactDistance(planeTravel, bounds.normalTravel * 0.001)
                - robotkit.collision.CollisionClearance.CONTACT_MARGIN;
              if (!(checkedTravel > 0)) throw "Probe approach has no static clearance interval before its named contact window";
              var blocked = prepare.motion.sensingCorridorViolation(cast checked.approachJoints,
                checked.direction, checkedTravel);
              if (blocked != null)
                throw 'Probe sensing corridor is blocked: ${blocked.a} against ${blocked.b}, ' +
                  '${blocked.distance} m < ${blocked.required} m';
              var request = checked.withPrecision(error, job.calibratedError, ContactProbeRunner.SEARCH_AGE_SECONDS);
              if (contactToolPrefix != null) request = request.withContactWindow(contactToolPrefix, stage.face.member);
              probes.push(request);
              // Retreat restores this proved configuration before the next search begins.
              from = cast checked.approachJoints;
            }
            return new ContactRegistrationStage(normal, stage.face.offset() * 0.001, probes);
        } catch (error:Dynamic) {
          var key = stage.face.name() + (discover ? ":global" : ":observed");
          if (!reported.exists(key)) {
            reported.set(key, true);
            reasons.push('${stage.face.name()}: ${Std.string(error)}');
          }
        }
      }
    }
    }
    throw reasons.length == 0 ? 'No CAD patch contains the measured uncertainty with reachable clear configurations for $count contact probes'
      : 'No checked reachable $count-contact CAD stage: ${reasons.join("; ")}';
  }

  static function samePattern(a:WeldProbeStage, b:WeldProbeStage):Bool {
    if (a.face != b.face || a.points.length != b.points.length) return false;
    for (i in 0...a.points.length) if (a.points[i].subtract(b.points[i]).length() > 1e-8) return false;
    return true;
  }

  static function pointKey(face:Int, point:Vector):String {
    var output = new BytesOutput();
    output.writeDouble(point.x);
    output.writeDouble(point.y);
    output.writeDouble(point.z);
    var bytes = output.getBytes();
    var result = face + ":";
    for (i in 0...24) result += bytes.get(i) + ":";
    return result;
  }
}
