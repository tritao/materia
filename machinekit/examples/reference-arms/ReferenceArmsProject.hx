import haxe.io.Bytes;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactData;

/** Select a prepared official arm at a common base mount, using vendor joint coordinates. */
class ReferenceArmsProject {
  public static function scene(id:String):SceneArtifactData {
    var robot = new ReferenceRobot(ReferenceAssetLocation.DIRECTORY + "/" + id);
    var scene = robot.scene;
    var state = new cadkit.modeling.AssemblyState(scene.assemblyDefinition);
    var ready = [0.0,-1,1.3,-1.5,-1,0];
    var index = 0;
    for (joint in robot.manifest.joints) if (joint.type != "fixed") {
      var value = ready[index++];
      if (joint.limits.lower != null && value < joint.limits.lower || joint.limits.upper != null && value > joint.limits.upper)
        value = 0.0;
      state.setJoint(joint.id,value);
    }
    scene.assemblyState = state.record();
    return scene;
  }
  public static function artifact(id:String):Bytes return SceneArtifact.encode(scene(id));
  public static function FR5WML():Bytes return artifact("FR5WML");
  public static function fairino10_v6():Bytes return artifact("fairino10_v6");
  public static function fairino16_v6():Bytes return artifact("fairino16_v6");
  public static function fairino20_v6():Bytes return artifact("fairino20_v6");
  public static function fairino30_v6():Bytes return artifact("fairino30_v6");
  public static function fairino3_mt_v6():Bytes return artifact("fairino3_mt_v6");
  public static function fairino3_v6():Bytes return artifact("fairino3_v6");
  public static function fairino5_v6():Bytes return artifact("fairino5_v6");
  public static function ur10():Bytes return artifact("ur10");
  public static function ur10e():Bytes return artifact("ur10e");
  public static function ur12e():Bytes return artifact("ur12e");
  public static function ur15():Bytes return artifact("ur15");
  public static function ur16e():Bytes return artifact("ur16e");
  public static function ur18():Bytes return artifact("ur18");
  public static function ur20():Bytes return artifact("ur20");
  public static function ur3():Bytes return artifact("ur3");
  public static function ur30():Bytes return artifact("ur30");
  public static function ur3e():Bytes return artifact("ur3e");
  public static function ur5():Bytes return artifact("ur5");
  public static function ur5e():Bytes return artifact("ur5e");
  public static function ur7e():Bytes return artifact("ur7e");
  public static function ur8long():Bytes return artifact("ur8long");
}
