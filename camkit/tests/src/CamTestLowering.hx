import camkit.CamProgram;
import cnckit.CncMachine;
import toolpathkit.motion.MachineBinding;
import toolpathkit.motion.ToolpathLoweringResult;
import toolpathkit.motion.ToolpathMotion;

/** Execution used only by CAM integration tests. */
class CamTestLowering {
  public static function lower(program:CamProgram,
      machine:CncMachine):ToolpathLoweringResult {
    var binding = new MachineBinding(machine.frameId, machine.xAxisId,
      machine.yAxisId, machine.zAxisId, machine.rapidSpeed,
      machine.initialPosition, machine.positionTolerance,
      machine.orientationTolerance, machine.maxBlendTurnAngleRadians);
    if (machine.travelLower != null && machine.travelUpper != null)
      binding.setTravelEnvelope(machine.travelLower, machine.travelUpper);
    return ToolpathMotion.lower(program.ops, binding);
  }
}
