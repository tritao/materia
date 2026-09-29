import cnckit.CncController;
import toolpathkit.motion.MachineBinding;
import toolpathkit.path.Point3;

/** CNC controller and physical binding for integration tests pending relocation. */
class MotionCncRig {
  public final controller:CncController = new CncController();
  public final binding:MachineBinding;
  public final start:Point3;

  public function new(frameId:String, xAxisId:String, yAxisId:String,
      zAxisId:String, rapidSpeed:Float, ?initialPosition:Array<Float>,
      ?positionTolerance:Float = 0.0005) {
    binding = new MachineBinding(frameId, xAxisId, yAxisId, zAxisId,
      rapidSpeed, positionTolerance);
    var initial = initialPosition == null ? [0.0, 0.0, 0.0] : initialPosition;
    start = new Point3(initial[0], initial[1], initial[2]);
  }
}
