package processkit;

import materia.project.SceneArtifact.SceneArtifactWeld;
import materia.project.SceneArtifact.SceneArtifactWeldPass;
import materia.project.SceneArtifact.SceneArtifactWeldSegment;
import materia.project.SceneArtifact.SceneArtifactTorchPose;
import processkit.skill.WeldPlan;
import processkit.skill.WeldPlan.WeldSegment;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;

/** One current scene pass interpreted identically by CAD planning and mission execution. */
class WeldScenePlan {
  public static function pass(weld:SceneArtifactWeld, pass:SceneArtifactWeldPass, frame:Transform3):WeldPlan {
    function pose(torch:SceneArtifactTorchPose):Transform3
      return frame.compose(new Transform3(new Vec3(torch.position[0], torch.position[1], torch.position[2]),
        new Quat(torch.rotation[0], torch.rotation[1], torch.rotation[2], torch.rotation[3])));
    var process = pass.process;
    // The open side of a corner is the bisector of its two faces' outward normals.
    function open(segment:materia.project.SceneArtifact.SceneArtifactWeldSegment):Vec3 {
      var a = segment.normals[0], b = segment.normals[1];
      return frame.rotation.rotate(new Vec3(a[0] + b[0], a[1] + b[1], a[2] + b[2]).normalized());
    }
    var parameters:processkit.skill.WeldPlan.WeldParameters = {wireSpeed: process.wireSpeed, voltage: process.voltage, travelSpeed: process.travelSpeed, approach: process.approach,
        startDwell: process.startDwell, craterDwell: process.craterDwell, burnback: process.burnback};
    var weave = pass.weave;
    if (weave != null) {
      var pattern:motionkit.path.WeavePattern = switch weave.pattern {
        case "sine": Sine;
        case "triangle": Triangle;
        case "zigzag": Zigzag;
        default: throw 'Unknown weave pattern "${weave.pattern}"';
      };
      parameters.weave = new motionkit.path.WeaveProfile(pattern, weave.amplitude, weave.cyclesPerMetre,
        process.travelSpeed, weave.edgeDwell);
    }
    var base = [for (segment in weld.path) new WeldSegment(pose(segment.start), pose(segment.stop), segment.seam, open(segment))];
    var normals = [for (segment in weld.path) [for (normal in segment.normals)
      frame.transformVector(new Vec3(normal[0], normal[1], normal[2]))]];
    return new WeldPlan(processkit.WeldPassPath.offset(base, normals, pass.offset[0], pass.offset[1]), parameters);
  }
}
