import cnckit.CncCompiler;
import cnckit.CncCompileResult;
import cnckit.CncController;
import cnckit.CncWriter;
import toolpathkit.motion.MachineBinding;
import toolpathkit.path.Point3;
import toolpathkit.path.ToolpathProgram;
import toolpathkit.setup.Setup;
import toolpathkit.setup.TravelEnvelope;
import toolpathkit.tool.ToolLibrary;

/** Controller and physical binding used by CAM integration tests. */
class CamTestRig {
  public final controller:CncController = new CncController();
  public final binding:MachineBinding;
  public var toolLibrary(get, never):ToolLibrary;

  public function new() {
    binding = new MachineBinding("work", "x", "y", "z", 0.2);
  }

  function get_toolLibrary():ToolLibrary return controller.toolLibrary;

  public function setTravelEnvelope(lower:Array<Float>, upper:Array<Float>):Void
    binding.setTravel(new TravelEnvelope(new Point3(lower[0], lower[1], lower[2]),
      new Point3(upper[0], upper[1], upper[2])));

  public function compileDetailed(source:String):CncCompileResult
    return CncCompiler.compileDetailed(source, controller, null, binding.travel);

  public function export(program:ToolpathProgram, setup:Setup):String {
    var setups = [setup];
    var ids = new Map<String, Bool>();
    ids.set(setup.id, true);
    for (op in program.ops) switch op {
      case SetSetup(id, _):
        if (!ids.exists(id)) {
          var offset = controller.workOffset(controller.gCodeForSetup(id));
          setups.push(new Setup(id, new Point3(offset[0], offset[1], offset[2])));
          ids.set(id, true);
        }
      case _:
    }
    return CncWriter.write(new ToolpathProgram(program.ops, program.tools, setups),
      controller, binding.travel);
  }
}
