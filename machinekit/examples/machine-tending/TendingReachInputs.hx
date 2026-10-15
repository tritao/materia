import machinekit.assembly.AssemblyPreview;
import machinekit.robotics.CobotArm;
import machinekit.robotics.CobotClass;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactData;

/** One invocation's reference assemblies and artifacts, shared by hashing and validation.
 * Custom study cells stay invocation-local; only the fixed reference inputs may use disk caching.
 */
class TendingReachInputs {
  public final cell:EnclosedBenchMill;
  public final source:CobotArm;
  public final tool:machinekit.robotics.DualArmGripperTool;
  public final assembly:machinekit.assembly.MachineAssembly;
  public final millScene:SceneArtifactData;
  public final armScene:SceneArtifactData;
  public final millBytes:haxe.io.Bytes;
  public final armBytes:haxe.io.Bytes;

  public function new(customCell:Null<EnclosedBenchMill>, persistent:Bool = false) {
    // A supplied cell may have runtime overrides: never reuse its disk artifact.
    var cache = new TendingGeometryCache(persistent && customCell == null);
    this.cell = customCell == null ? new EnclosedBenchMill() : customCell;
    tool = TendingReach.tendingTool();
    source = new CobotArm(Reach1300, tool);
    assembly = TendingReach.armAssembly(source);
    var mill = cache.scene("reach-mill", () -> AssemblyPreview.scene(this.cell, "reach-mill"));
    var arm = cache.scene("reach-arm", () -> AssemblyPreview.scene(assembly, "reach-arm"));
    millScene = mill.data; millBytes = mill.bytes;
    armScene = arm.data; armBytes = arm.bytes;
  }
}
