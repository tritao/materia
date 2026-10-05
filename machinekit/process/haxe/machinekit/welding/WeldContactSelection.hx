package machinekit.welding;

import cadkit.modeling.Vector;
import machinekit.welding.WeldProbeGeometry;
import machinekit.welding.WeldProbePatterns;
import materia.project.SceneContactRegistration.SceneContactWork;
import machinekit.welding.WeldProbeParkingBounds.WeldProbeRegionBounds;
import machinekit.welding.WeldProbeUncertainty.WeldProbeObservedBounds;
import processkit.ProbeMotionPlanner;
import processkit.ProbePosePlanner;
import processkit.ContactProbeRunner.ContactProbeRequest;
import processkit.perception.ContactPoseEnvelope;
import processkit.perception.ContactRegistrationSequence;
import processkit.perception.ContactRegistrationSequence.ContactRegistrationStage;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/** Bridge saved nominal CAD patches to measured registration and checked arm probe preparation. */
class WeldContactSelection {
  final job:SceneContactWork;
  final geometry:WeldProbeGeometry;
  final prepare:ProbePosePlanner;
  final positions:Void->Array<Float>;
  public function new(job:SceneContactWork, planner:ProbeMotionPlanner, positions:Void->Array<Float>) {
    if (job == null || planner == null || positions == null) throw "Contact selection needs a saved CAD job and observed arm joints";
    this.job = job; this.positions = positions;
    geometry = WeldProbeGeometry.fromContactFaces(job.faces, 0.001);
    prepare = new ProbePosePlanner(planner);
  }
  static function metres(point:Vector):Vec3 return new Vec3(point.x * 0.001, point.y * 0.001, point.z * 0.001);
  static function direction(point:Vector):Vec3 return new Vec3(point.x, point.y, point.z);

  public function sequence(nominalRootWork:Transform3):ContactRegistrationSequence {
    var envelope = new ContactPoseEnvelope(nominalRootWork, Vec3.fromArray(job.translation),
      Vec3.fromArray(job.rotation), job.measurementError);
    return new ContactRegistrationSequence(envelope, select);
  }

  function select(count:Int, normals:Array<Vec3>, envelope:ContactPoseEnvelope):ContactRegistrationStage {
    var estimate = envelope.estimate();
    var provider = new WeldProbeObservedBounds((face, point) -> {
      var bounds = envelope.region(metres(point), direction(face.normal), direction(face.u), direction(face.v), estimate);
      return new WeldProbeRegionBounds(bounds.halfU * 1000, bounds.halfV * 1000,
        bounds.normalTravel * 1000, bounds.tiltU, bounds.tiltV);
    });
    var start = positions();
    var reasons:Array<String> = [];
    for (divisions in [9, 17, 33]) {
      var previous = [for (normal in normals) new Vector(normal.x, normal.y, normal.z)];
      var geometric = WeldProbePatterns.stages(geometry, count, previous, provider, 3, divisions);
      // Rank by geometric observability, then prove one face at a time. Unused faces need no IK solves.
      for (candidate in geometric) {
        var stages = WeldProbePatterns.stages(geometry, count, previous, provider, 3, divisions,
          (face, point, bounds) -> prepare.hasClearApproachConfiguration(estimate.transformPoint(metres(point)),
            estimate.rotation.rotate(direction(face.normal)), bounds.normalTravel * 0.001, start), candidate.face);
        for (stage in stages) {
          try {
            var probes:Array<ContactProbeRequest> = [];
            var normal = direction(stage.face.normal);
            for (point in stage.points) {
              var bounds = provider.region(stage.face, point);
              probes.push(prepare.prepare(estimate.transformPoint(metres(point)), estimate.rotation.rotate(normal),
                bounds.normalTravel * 0.001, start, job.contactOffset));
            }
            return new ContactRegistrationStage(normal, stage.face.offset() * 0.001, probes);
          } catch (error:Dynamic) reasons.push('${stage.face.name()}: ${Std.string(error)}');
        }
      }
    }
    throw reasons.length == 0 ? 'No CAD patch contains the measured uncertainty with reachable clear configurations for $count contact probes'
      : 'No checked reachable $count-contact CAD stage: ${reasons.join("; ")}';
  }
}
