import toolpathkit.motion.ToolpathLoweringResult;
import toolpathkit.motion.ToolpathMotion;

/** Execution used only by CAM integration tests. */
class CamTestLowering {
  public static function lower(program:toolpathkit.path.ToolpathProgram,
      machine:CamTestRig):ToolpathLoweringResult {
    return ToolpathMotion.lower(program, machine.binding);
  }
}
