import cnckit.CncController;
import cnckit.CncDialect;
import cnckit.CncCompiler;
import cnckit.CncCompileResult;
import toolpathkit.motion.MachineBinding;
import toolpathkit.path.Point3;
import toolpathkit.setup.TravelEnvelope;
import toolpathkit.tool.ToolLibrary;

/** Test fixture that keeps controller settings separate from physical binding. */
class CncTestRig {
  public final controller:CncController;
  public final binding:MachineBinding;
  public final start:Point3;
  public var toolLibrary(get, never):ToolLibrary;

  public function new(frameId:String, xAxisId:String, yAxisId:String,
      zAxisId:String, rapidSpeed:Float, ?initialPosition:Array<Float>,
      ?positionTolerance:Float = 0.0005,
      ?orientationTolerance:Float = 0.02,
      ?dialect:CncDialect = LinuxCnc,
      ?maxBlendTurnAngleRadians:Float = Math.PI * 5.0 / 6.0) {
    controller = new CncController(dialect);
    binding = new MachineBinding(frameId, xAxisId, yAxisId, zAxisId,
      rapidSpeed, positionTolerance, orientationTolerance,
      maxBlendTurnAngleRadians);
    var initial = initialPosition == null ? [0.0, 0.0, 0.0] : initialPosition;
    if (initial.length != 3) throw "CNC test start needs three coordinates";
    start = new Point3(initial[0], initial[1], initial[2]);
  }

  function get_toolLibrary():ToolLibrary return controller.toolLibrary;

  public function setTravelEnvelope(lower:Array<Float>, upper:Array<Float>):Void
    binding.setTravel(new TravelEnvelope(new Point3(lower[0], lower[1], lower[2]),
      new Point3(upper[0], upper[1], upper[2])));

  public function compileDetailed(source:String):CncCompileResult
    return CncCompiler.compileDetailed(source, controller, start, binding.travel);
}
