import cadkit.modeling.Part;
import cadkit.threads.SurfaceDetail.DisplayBinding;
import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentRecipeSupport as R;
import machinekit.component.ComponentValues;
import machinekit.component.Solids;
import machinekit.hmi.DeviceBodySpec;
import machinekit.hmi.DeviceJointSpec;
import machinekit.hmi.HmiPart;
import machinekit.pendant.PendantMachine;
import materia.assembly.AssemblyDefinition.AssemblyJointLimits;
import materia.assembly.AssemblyFrames;

/** A carriage on a round rail, sliding along the device's z axis: one axis of the jog stage. */
class JogSlider extends HmiPart {
  public function new(?v:ComponentValues) {
    super("jog-slider", "jog stage slider", "aluminium 6061", [
      HmiPart.basisParameter("assumed"),
      R.length("travel", 120),
      R.length("railRadius", 5),
      R.length("carriageRadius", 11),
      R.length("carriageLength", 30),
      R.scalar("massGrams", 300, 1)
    ], v);
    positive(["travel", "railRadius", "carriageLength", "massGrams"]);
    require(n("carriageRadius") > n("railRadius"), "the carriage is wider than the rail it slides on");
  }

  override public function bodies():Null<Array<DeviceBodySpec>> {
    var travel = n("travel"), r = n("railRadius"), big = n("carriageRadius"), length = n("carriageLength");
    return [
      new DeviceBodySpec("rail", "slider rail", "stainless A2-70", function() return Solids.named(Part.cylinderSpan(r, 0.0, travel + length), "rail")),
      new DeviceBodySpec("carriage", "slider carriage", "aluminium 6061", function() return Solids.named(
        Part.revolve([{r: r, z: 0.0}, {r: big, z: 0.0}, {r: big, z: length}, {r: r, z: length}]),
        "carriage"
      ))
    ];
  }

  /** The carriage slides from the rail's start to the far end of its travel. */
  override public function joints():Array<DeviceJointSpec> {
    var limits:AssemblyJointLimits = {lower: 0.0, upper: n("travel"), velocity: null, effort: null};
    return [new DeviceJointSpec("slide", Prismatic, "rail", "carriage", limits, 0.0)];
  }
}

/**
 * Three sliders along the world's x, y and z, side by side: a demonstration machine for the pendant, whose wheel jogs the one the
 * axis switch selects. They are independent (not stacked), which is enough to see a jog move the right joint and stop at its limit.
 */
class JogStage {
  /** The operator profile of this machine (see `ReferenceOperatorProfiles.stage`). */
  public static inline var PROFILE:String = "stage";

  /** The stage's joint for each of the profile's axes, as `assembly` names them once it is included as `stage`. */
  public static function joints():Map<String, String> return ["x" => "stage/x/slide", "y" => "stage/y/slide", "z" => "stage/z/slide"];

  /** The pendant of this machine: the reference pendant, serving the stage's profile and jogging its joints. */
  public static function machine():PendantMachine return PendantMachine.moving(PROFILE, joints());

  /** Adds the stage beside the pendant in `assembly`, as the subassembly `stage`. */
  public static function addTo(assembly:MachineAssembly):Void {
    var stage = new MachineAssembly();
    // A slider's axis is its z. Turn one onto x (about y) and one onto y (about x); the third stays on z.
    var quarter = Math.PI / 2;
    stage.include("x", new JogSlider().expand(), AssemblyFrames.compose(AssemblyFrames.translation(0, -60, 0), AssemblyFrames.turnY(quarter)));
    stage.include("y", new JogSlider().expand(), AssemblyFrames.compose(AssemblyFrames.translation(0, 0, 0), AssemblyFrames.turnX(-quarter)));
    stage.include("z", new JogSlider().expand(), AssemblyFrames.translation(0, 60, 0));
    assembly.include("stage", stage, AssemblyFrames.translation(160, 0, 0));
  }
}
